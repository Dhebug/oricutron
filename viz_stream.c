/*
**  Oricutron
**  Copyright (C) 2009-2014 Peter Gordon
**
**  This program is free software; you can redistribute it and/or
**  modify it under the terms of the GNU General Public License
**  as published by the Free Software Foundation, version 2
**  of the License.
**
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU General Public License for more details.
**
**  You should have received a copy of the GNU General Public License
**  along with this program; if not, write to the Free Software
**  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
**  Memory access heatmap visualization stream
**
**  Pushes binary frames over TCP to a connected client (VS Code webview).
**  Port is auto-derived as gdb_port + 1 (see viz_init).
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;
#define VIZ_INVALID_SOCKET INVALID_SOCKET
#define VIZ_SOCKET_ERROR   SOCKET_ERROR
#define viz_closesocket    closesocket
#define viz_would_block()  (WSAGetLastError() == WSAEWOULDBLOCK)
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#define VIZ_INVALID_SOCKET (-1)
#define VIZ_SOCKET_ERROR   (-1)
#define viz_closesocket    close
#define viz_would_block()  (errno == EWOULDBLOCK || errno == EAGAIN)
typedef int SOCKET;
#endif

#include "system.h"
#include "6502.h"
#include "via.h"
#include "8912.h"
#include "gui.h"
#include "disk.h"
#include "monitor.h"
#include "6551.h"
#include "machine.h"
#include "ula.h"
#include "viz_stream.h"

/* ------------------------------------------------------------------ */
/* Global heatmap arrays                                               */
/* ------------------------------------------------------------------ */

SDL_bool viz_tracking = SDL_FALSE;
SDL_bool viz_suppress = SDL_FALSE;
Uint8 viz_heat_read[65536];
Uint8 viz_heat_write[65536];
Uint8 viz_heat_ula[65536];

/* ------------------------------------------------------------------ */
/* Binary frame format                                                 */
/* ------------------------------------------------------------------ */

/*
  Binary frame - v2 (current). Little-endian, variable length.

  Header (16 bytes):
    0   4   Magic "OVIC" (0x4F564943)
    4   4   Frame counter (uint32)
    8   1   ROMDIS state (0/1)
    9   1   Video mode (oric->vid_mode)
    10  2   Video address (oric->vid_addr)
    12  2   Charset address
    14  2   Version (= 2)

  Heat deltas (variable): three run-lists in order read, write, ULA. Each is
    u16 nRuns, then nRuns x (u16 start, u16 len)
  A run means addresses [start, start+len) were accessed this frame (full heat).
  The emulator clears its touched flags after each frame; the CLIENT decays
  (-VIZ_DECAY_RATE per frame) and applies these runs. No keyframe: a decaying
  view needs no baseline, so a fresh client converges within the decay window.

  Screen block (fixed, follows the heat deltas):
    scr       53760   Screen buffer (240*224 colour indices 0-7)
    vidbases  8       vidbases[0..3] (4 x uint16)
    vidram    8000    Video RAM main area (vid_addr .. vid_addr+7999)
    vidram    120     Video RAM bottom 3 rows (vidbases[2] .. vidbases[2]+119)

  Legacy: v0 = header + 3x65536 full heatmaps; v1 = v0 + the screen block. The
  consumer still parses those; the emulator now emits only v2.
*/

#define VIZ_FRAME_HEADER  16
#define VIZ_VERSION       2   /* v2: heat sent as per-frame DELTAS (run-lists); client decays */
#define VIZ_SCR_SIZE      (240 * 224)           /* 53760 */
#define VIZ_VIDBASES_SIZE (4 * 2)               /* 8 */
#define VIZ_VIDRAM_MAIN   8000
#define VIZ_VIDRAM_BOTTOM 120
#define VIZ_V1_EXTRA      (VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE + VIZ_VIDRAM_MAIN + VIZ_VIDRAM_BOTTOM)

/* Hash of the rendered framebuffer (oric->scr) in the last frame we built. Lets us
   detect when the picture actually changed — used to push a fresh frame while paused
   (single-stepping) without spamming unchanged screens. */
static Uint32 viz_last_scr_hash = 0;

/* FNV-1a over the rendered framebuffer. Cheap; catches any pixel change, whatever the
   cause (character, redefined font, attribute, or a resolution switch that re-renders). */
static Uint32 viz_hash_scr(struct machine *oric)
{
  Uint32 h = 2166136261u;
  int i;
  if(!oric->scr) return 0;
  for(i = 0; i < VIZ_SCR_SIZE; i++) { h ^= oric->scr[i]; h *= 16777619u; }
  return h;
}

/* Each heat array is emitted as a run-list: [u16 nRuns][nRuns x (u16 start, u16 len)],
   covering the non-zero spans touched since the last push. Worst case is an
   alternating touched/untouched pattern = 32768 single-byte runs. */
#define VIZ_HEAT_MAX_BYTES (2 + 32768 * 4)
#define VIZ_FRAME_BUF_MAX  (VIZ_FRAME_HEADER + VIZ_HEAT_MAX_BYTES * 3 + VIZ_V1_EXTRA)
#define VIZ_FRAME_SKIP    1   /* push every emulated frame (~50fps) — heat is now tiny */
#define VIZ_DECAY_RATE    8   /* reference rate; decay is now client-side — the
                                 consumer's HEAT_DECAY (extension.js) MUST match this */

/* ------------------------------------------------------------------ */
/* Internal state                                                      */
/* ------------------------------------------------------------------ */

static struct {
  SOCKET listen_sock;
  SOCKET client_sock;
  SDL_bool initialized;
  SDL_bool wsa_initialized;
  int port;
  int frame_counter;
  int skip_counter;
  unsigned char *frame_buf;   /* VIZ_FRAME_BUF_MAX bytes, allocated once */
  int frame_len;              /* Actual length of the current frame (variable in v2) */
  int send_offset;            /* Resume offset for partial frame sends */
  SDL_bool frame_pending;     /* True if a partially-sent frame is in progress */
  unsigned char rxbuf[256];   /* Uplink (VS Code -> emulator) receive buffer */
  int rxlen;                  /* Bytes currently buffered in rxbuf */
} viz = {
  VIZ_INVALID_SOCKET,
  VIZ_INVALID_SOCKET,
  SDL_FALSE,
  SDL_FALSE,
  0,
  0,
  0,
  NULL,       /* frame_buf */
  0,          /* frame_len */
  0,          /* send_offset */
  SDL_FALSE   /* frame_pending */
};

/* ------------------------------------------------------------------ */
/* Socket helpers (mirror gdb_stub.c patterns)                         */
/* ------------------------------------------------------------------ */

static SDL_bool viz_socket_init(void)
{
#ifdef WIN32
  if(!viz.wsa_initialized)
  {
    WSADATA wsadata;
    if(WSAStartup(MAKEWORD(2,2), &wsadata) != 0)
      return SDL_FALSE;
    viz.wsa_initialized = SDL_TRUE;
  }
#endif
  return SDL_TRUE;
}

static void viz_socket_cleanup(void)
{
#ifdef WIN32
  if(viz.wsa_initialized)
  {
    WSACleanup();
    viz.wsa_initialized = SDL_FALSE;
  }
#endif
}

static void viz_set_nonblocking(SOCKET sock)
{
#ifdef WIN32
  u_long imode = 1;
  ioctlsocket(sock, FIONBIO, &imode);
#else
  fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
#endif
}

static void viz_close_client(void)
{
  if(viz.client_sock != VIZ_INVALID_SOCKET)
  {
    viz_closesocket(viz.client_sock);
    viz.client_sock = VIZ_INVALID_SOCKET;
    printf("VIZ: client disconnected\n");
  }
}

/* ------------------------------------------------------------------ */
/* Frame building and sending                                          */
/* ------------------------------------------------------------------ */

static void viz_write_u16_le(unsigned char *dst, Uint16 val)
{
  dst[0] = (unsigned char)(val & 0xff);
  dst[1] = (unsigned char)((val >> 8) & 0xff);
}

static void viz_write_u32_le(unsigned char *dst, Uint32 val)
{
  dst[0] = (unsigned char)(val & 0xff);
  dst[1] = (unsigned char)((val >> 8) & 0xff);
  dst[2] = (unsigned char)((val >> 16) & 0xff);
  dst[3] = (unsigned char)((val >> 24) & 0xff);
}

/* Emit heat[] as a run-list into p: [u16 nRuns][nRuns x (u16 start, u16 len)]
   covering the non-zero spans (addresses touched since the last push). Returns
   bytes written. Does NOT clear heat[] (the caller does, once). */
static int viz_emit_runs(unsigned char *p, const Uint8 *heat)
{
  int i = 0, out = 2, nruns = 0;
  while(i < 65536)
  {
    if(heat[i])
    {
      int start = i, len;
      while(i < 65536 && heat[i] && (i - start) < 0xffff) i++;
      len = i - start;
      p[out++] = (unsigned char)(start & 0xff);
      p[out++] = (unsigned char)((start >> 8) & 0xff);
      p[out++] = (unsigned char)(len & 0xff);
      p[out++] = (unsigned char)((len >> 8) & 0xff);
      nruns++;
    }
    else
    {
      i++;
    }
  }
  p[0] = (unsigned char)(nruns & 0xff);
  p[1] = (unsigned char)((nruns >> 8) & 0xff);
  return out;
}

/* Build a v2 frame into viz.frame_buf and mark it pending.
   Layout: 16-byte header, then 3 heat-delta run-lists (read/write/ula), then the
   screen block (screen + vidbases + vidram). The heat arrays are per-frame
   "touched" flags (set to 255 on access) and are CLEARED here after emission —
   decay is done client-side. Frame length is variable (viz.frame_len). */
static void viz_build_frame(struct machine *oric)
{
  unsigned char *p;
  Uint16 charset_addr;
  int i, off;
  Uint16 vid_addr, vb2_addr;

  if(!viz.frame_buf) return;

  p = viz.frame_buf;

  /* Header */
  p[0] = 'O'; p[1] = 'V'; p[2] = 'I'; p[3] = 'C';
  viz_write_u32_le(p + 4, (Uint32)viz.frame_counter);
  p[8] = oric->romdis ? 1 : 0;
  p[9] = (unsigned char)(oric->vid_mode & 0xff);
  viz_write_u16_le(p + 10, oric->vid_addr);
  charset_addr = (oric->vid_mode & 4) ? oric->vidbases[1] : oric->vidbases[3];
  viz_write_u16_le(p + 12, charset_addr);
  viz_write_u16_le(p + 14, VIZ_VERSION);

  /* Heat deltas (run-lists), then clear the touched flags for the next frame */
  off = VIZ_FRAME_HEADER;
  off += viz_emit_runs(p + off, viz_heat_read);
  off += viz_emit_runs(p + off, viz_heat_write);
  off += viz_emit_runs(p + off, viz_heat_ula);
  memset(viz_heat_read,  0, 65536);
  memset(viz_heat_write, 0, 65536);
  memset(viz_heat_ula,   0, 65536);

  /* Screen block (unchanged from v1), placed after the variable-length heat */
  if(oric->scr)
    memcpy(p + off, oric->scr, VIZ_SCR_SIZE);
  else
    memset(p + off, 0, VIZ_SCR_SIZE);
  viz_last_scr_hash = viz_hash_scr(oric);  /* remember the picture we just sent */

  for(i = 0; i < 4; i++)
    viz_write_u16_le(p + off + VIZ_SCR_SIZE + i * 2, oric->vidbases[i]);

  vid_addr = oric->vid_addr;
  if(vid_addr + VIZ_VIDRAM_MAIN <= 65536)
    memcpy(p + off + VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE, oric->mem + vid_addr, VIZ_VIDRAM_MAIN);
  else
  {
    int first = 65536 - vid_addr;
    memcpy(p + off + VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE, oric->mem + vid_addr, first);
    memset(p + off + VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE + first, 0, VIZ_VIDRAM_MAIN - first);
  }

  vb2_addr = oric->vidbases[2];
  if(vb2_addr + VIZ_VIDRAM_BOTTOM <= 65536)
    memcpy(p + off + VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE + VIZ_VIDRAM_MAIN, oric->mem + vb2_addr, VIZ_VIDRAM_BOTTOM);
  else
  {
    int first = 65536 - vb2_addr;
    memcpy(p + off + VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE + VIZ_VIDRAM_MAIN, oric->mem + vb2_addr, first);
    memset(p + off + VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE + VIZ_VIDRAM_MAIN + first, 0, VIZ_VIDRAM_BOTTOM - first);
  }

  off += VIZ_V1_EXTRA;

  viz.frame_len = off;
  viz.send_offset = 0;
  viz.frame_pending = SDL_TRUE;
  viz.frame_counter++;
}

/* Try to send as much of the pending frame as possible.
   Returns SDL_TRUE when the entire frame has been sent. */
static SDL_bool viz_flush_frame(void)
{
  while(viz.send_offset < viz.frame_len)
  {
    int n = send(viz.client_sock,
                 (const char *)(viz.frame_buf + viz.send_offset),
                 viz.frame_len - viz.send_offset, 0);
    if(n > 0)
    {
      viz.send_offset += n;
    }
    else if(n == 0)
    {
      viz_close_client();
      viz.frame_pending = SDL_FALSE;
      return SDL_FALSE;
    }
    else
    {
      if(viz_would_block())
        return SDL_FALSE; /* Will resume on next poll */
      viz_close_client();
      viz.frame_pending = SDL_FALSE;
      return SDL_FALSE;
    }
  }

  viz.frame_pending = SDL_FALSE;
  return SDL_TRUE;
}

/* ------------------------------------------------------------------ */
/* Uplink: input injection (VS Code -> emulator)                       */
/* ------------------------------------------------------------------ */

/*
  Uplink message framing (client -> emulator), on the same socket:
    [opcode:u8][len:u8][payload:len bytes]

  0x01 KEY          payload = [keyid:u8][down:u8]   (down: 1=press 0=release)
  0x02 RELEASE_ALL  payload = (none)                (panic release on focus loss)

  keyid is SDL-version-agnostic: 0x20..0x7e are passed through as the keysym
  (SDLK == ASCII for alphanumerics/most punctuation in both SDL1 and SDL2);
  0x80+ are special keys mapped to SDLK_* here so the extension needs no SDL
  knowledge.
*/

/* Map a portable keyid to an SDL keysym understood by ay_keypress()/keytab.
   Returns 0 for unknown ids (ay_keypress ignores key 0). */
static SDL_COMPAT_KEY viz_map_key(unsigned char id)
{
  switch(id)
  {
    case 0x80: return SDLK_UP;
    case 0x81: return SDLK_DOWN;
    case 0x82: return SDLK_LEFT;
    case 0x83: return SDLK_RIGHT;
    case 0x84: return SDLK_RETURN;
    case 0x85: return SDLK_ESCAPE;
    case 0x86: return SDLK_SPACE;
    case 0x87: return SDLK_BACKSPACE;
    case 0x88: return SDLK_LSHIFT;
    case 0x89: return SDLK_LCTRL;
    case 0x8b: return SDLK_TAB;
    default:
      /* Printable ASCII maps straight through to the matching SDLK. */
      if(id >= 0x20 && id < 0x7f)
        return (SDL_COMPAT_KEY)id;
      return 0;
  }
}

/* Non-blocking read of inbound uplink messages, dispatching each complete
   frame.  Detects client disconnect (recv==0). */
static void viz_process_input(struct machine *oric)
{
  int n, off;

  n = recv(viz.client_sock,
           (char *)(viz.rxbuf + viz.rxlen),
           (int)(sizeof(viz.rxbuf) - (size_t)viz.rxlen), 0);
  if(n == 0)
  {
    viz_close_client();
    return;
  }
  if(n < 0)
  {
    if(!viz_would_block())
      viz_close_client();
    return;
  }

  viz.rxlen += n;

  off = 0;
  while(viz.rxlen - off >= 2)
  {
    unsigned char op  = viz.rxbuf[off];
    unsigned char len = viz.rxbuf[off + 1];
    const unsigned char *pl = viz.rxbuf + off + 2;

    if(viz.rxlen - off - 2 < len)
      break;  /* wait for the rest of the payload */

    switch(op)
    {
      case 0x01:  /* KEY */
        if(len >= 2)
        {
          SDL_COMPAT_KEY k = viz_map_key(pl[0]);
          if(k)
            ay_keypress(&oric->ay, k, pl[1] ? SDL_TRUE : SDL_FALSE);
        }
        break;

      case 0x02:  /* RELEASE_ALL - clear the whole matrix (self-heals on next scan) */
      {
        int i;
        for(i = 0; i < 8; i++)
          oric->ay.keystates[i] = SDL_FALSE;
        break;
      }

      default:
        break;  /* skip unknown opcodes */
    }

    off += 2 + len;
  }

  if(off > 0)
  {
    memmove(viz.rxbuf, viz.rxbuf + off, (size_t)(viz.rxlen - off));
    viz.rxlen -= off;
  }

  /* Overflow guard: a full buffer with no complete message = desync, resync. */
  if(viz.rxlen == (int)sizeof(viz.rxbuf))
    viz.rxlen = 0;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

SDL_bool viz_init(struct machine *oric)
{
  struct sockaddr_in addr;
  int on = 1;
  int port;

  if(viz.initialized)
    return SDL_TRUE;

  if(oric->gdb_port <= 0)
    return SDL_FALSE;

  /* Viz port = gdb port + 1, so the two sit together in a port scanner. The VS Code
     extension auto-picks a free gdb port and verifies gdb+1 is free too, so there is
     no collision. MUST match VIZ_PORT_OFFSET in the VS Code extension. */
  port = oric->gdb_port + 1;

  if(!viz_socket_init())
  {
    printf("VIZ: failed to initialize sockets\n");
    return SDL_FALSE;
  }

  viz.listen_sock = socket(AF_INET, SOCK_STREAM, 0);
  if(viz.listen_sock == VIZ_INVALID_SOCKET)
  {
    printf("VIZ: failed to create socket\n");
    return SDL_FALSE;
  }

  setsockopt(viz.listen_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&on, sizeof(on));
  viz_set_nonblocking(viz.listen_sock);

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons((unsigned short)port);

  if(bind(viz.listen_sock, (struct sockaddr *)&addr, sizeof(addr)) == VIZ_SOCKET_ERROR)
  {
    printf("VIZ: failed to bind to port %d\n", port);
    viz_closesocket(viz.listen_sock);
    viz.listen_sock = VIZ_INVALID_SOCKET;
    return SDL_FALSE;
  }

  if(listen(viz.listen_sock, 1) == VIZ_SOCKET_ERROR)
  {
    printf("VIZ: failed to listen on port %d\n", port);
    viz_closesocket(viz.listen_sock);
    viz.listen_sock = VIZ_INVALID_SOCKET;
    return SDL_FALSE;
  }

  /* Allocate frame buffer (sized for the worst-case heat run-lists + screen) */
  viz.frame_buf = (unsigned char *)malloc(VIZ_FRAME_BUF_MAX);
  if(!viz.frame_buf)
  {
    printf("VIZ: failed to allocate frame buffer\n");
    viz_closesocket(viz.listen_sock);
    viz.listen_sock = VIZ_INVALID_SOCKET;
    return SDL_FALSE;
  }

  viz.port = port;
  viz.initialized = SDL_TRUE;
  viz.client_sock = VIZ_INVALID_SOCKET;
  viz.frame_counter = 0;
  viz.skip_counter = 0;
  viz_tracking = SDL_TRUE;

  /* Clear heatmap arrays */
  memset(viz_heat_read, 0, sizeof(viz_heat_read));
  memset(viz_heat_write, 0, sizeof(viz_heat_write));
  memset(viz_heat_ula, 0, sizeof(viz_heat_ula));

  printf("VIZ: heatmap server listening on port %d\n", port);
  return SDL_TRUE;
}

void viz_shutdown(void)
{
  if(!viz.initialized)
    return;

  viz_tracking = SDL_FALSE;
  viz_close_client();

  if(viz.listen_sock != VIZ_INVALID_SOCKET)
  {
    viz_closesocket(viz.listen_sock);
    viz.listen_sock = VIZ_INVALID_SOCKET;
  }

  if(viz.frame_buf)
  {
    free(viz.frame_buf);
    viz.frame_buf = NULL;
  }

  viz_socket_cleanup();
  viz.initialized = SDL_FALSE;

  printf("VIZ: server shut down\n");
}

void viz_poll(struct machine *oric, SDL_bool running)
{
  if(!viz.initialized)
    return;

  /* Accept new connections */
  if(viz.client_sock == VIZ_INVALID_SOCKET)
  {
    struct sockaddr_in client_addr;
    socklen_t addrlen = sizeof(client_addr);
    SOCKET newsock = accept(viz.listen_sock, (struct sockaddr *)&client_addr, &addrlen);
    if(newsock != VIZ_INVALID_SOCKET)
    {
      viz.client_sock = newsock;
      viz_set_nonblocking(viz.client_sock);
      viz.frame_counter = 0;
      viz.skip_counter = 0;
      viz.frame_pending = SDL_FALSE;
      viz.send_offset = 0;
      printf("VIZ: client connected\n");

      /* No keyframe: the heatmap is a decaying view, so clear the touched flags
         (which accumulated while no client was attached) before the first build.
         The initial frame then carries an empty heat delta plus the current
         screen — the client converges within the decay window as code runs. */
      memset(viz_heat_read,  0, 65536);
      memset(viz_heat_write, 0, 65536);
      memset(viz_heat_ula,   0, 65536);
      viz_build_frame(oric);
      viz_flush_frame();
    }
    return;
  }

  /* Read inbound uplink messages (keyboard/input) every tick for low latency,
     even while paused or mid-frame-send.  Also detects client disconnect. */
  viz_process_input(oric);
  if(viz.client_sock == VIZ_INVALID_SOCKET)
    return;

  /* If a partial frame is still being sent, finish it first */
  if(viz.frame_pending)
  {
    viz_flush_frame();
    return;  /* Don't build a new frame until this one is done */
  }

  /* Paused (e.g. single-stepping in the debugger): the emulator isn't running frames,
     but the rendered picture can still change — a stepped write to screen RAM or the
     charset, or a resolution attribute that alters the next render. Push a fresh frame
     whenever the framebuffer differs from the one we last sent, so the client's screen
     view stays live; skip it when nothing changed (the common case) to avoid spamming
     the full screen. */
  if(!running)
  {
    if(oric->scr && viz_hash_scr(oric) != viz_last_scr_hash)
    {
      viz_build_frame(oric);
      viz_flush_frame();
    }
    return;
  }

  /* Throttle: only build a new frame every VIZ_FRAME_SKIP calls */
  viz.skip_counter++;
  if(viz.skip_counter < VIZ_FRAME_SKIP)
    return;
  viz.skip_counter = 0;

  /* Liveness is now handled by viz_process_input above (detects recv==0). */

  /* Build new frame and start sending */
  viz_build_frame(oric);
  viz_flush_frame();
}

void viz_frame_decay(void)
{
  /* v2: decay is done client-side (the emulator now sends per-frame access
     deltas and clears its touched flags in viz_build_frame). Kept as a no-op so
     the main-loop call site (once_per_frame) is undisturbed. */
}
