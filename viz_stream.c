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
**  Port is auto-derived as gdb_port + 1.
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
  Offset  Size     Field
  0       4        Magic "OVIC" (0x4F564943)
  4       4        Frame counter (uint32 LE)
  8       1        ROMDIS state (0 or 1)
  9       1        Video mode (oric->vid_mode)
  10      2        Video address (oric->vid_addr, uint16 LE)
  12      2        Charset address (uint16 LE)
  14      2        Version (0=v0 heatmap-only, 1=v1 with screen)
  16      65536    Read heatmap
  65552   65536    Write heatmap
  131088  65536    ULA heatmap
  --- v1 appended data (when version >= 1) ---
  196624  53760    Screen buffer (oric->scr, 240*224 color indices 0-7)
  250384  8        vidbases[0..3] (4 x uint16 LE)
  250392  8000     Video RAM main area (vid_addr .. vid_addr+7999)
  258392  120      Video RAM bottom 3 rows (vidbases[2] .. vidbases[2]+119)
  Total v0: 196624 bytes
  Total v1: 258512 bytes
*/

#define VIZ_FRAME_HEADER  16
#define VIZ_FRAME_SIZE    (VIZ_FRAME_HEADER + 65536 * 3)
#define VIZ_VERSION       1
#define VIZ_SCR_SIZE      (240 * 224)           /* 53760 */
#define VIZ_VIDBASES_SIZE (4 * 2)               /* 8 */
#define VIZ_VIDRAM_MAIN   8000
#define VIZ_VIDRAM_BOTTOM 120
#define VIZ_V1_EXTRA      (VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE + VIZ_VIDRAM_MAIN + VIZ_VIDRAM_BOTTOM)
#define VIZ_FRAME_SIZE_V1 (VIZ_FRAME_SIZE + VIZ_V1_EXTRA)
#define VIZ_FRAME_SKIP    3   /* Push every N emulated frames (~16.7fps at 50Hz) */
#define VIZ_DECAY_RATE    8   /* Subtracted per emulated frame */

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
  unsigned char *frame_buf;   /* VIZ_FRAME_SIZE_V1 bytes, allocated once */
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
  NULL,
  0,
  SDL_FALSE
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

/* Build frame into viz.frame_buf, mark as pending for sending */
static void viz_build_frame(struct machine *oric)
{
  unsigned char *p;
  Uint16 charset_addr;
  int i;
  Uint16 vid_addr, vb2_addr;

  if(!viz.frame_buf) return;

  p = viz.frame_buf;

  /* Magic */
  p[0] = 'O'; p[1] = 'V'; p[2] = 'I'; p[3] = 'C';

  /* Frame counter */
  viz_write_u32_le(p + 4, (Uint32)viz.frame_counter);

  /* ROMDIS */
  p[8] = oric->romdis ? 1 : 0;

  /* Video mode */
  p[9] = (unsigned char)(oric->vid_mode & 0xff);

  /* Video address */
  viz_write_u16_le(p + 10, oric->vid_addr);

  /* Charset address */
  charset_addr = (oric->vid_mode & 4) ? oric->vidbases[1] : oric->vidbases[3];
  viz_write_u16_le(p + 12, charset_addr);

  /* Version */
  viz_write_u16_le(p + 14, VIZ_VERSION);

  /* Copy heatmap arrays */
  memcpy(p + VIZ_FRAME_HEADER, viz_heat_read, 65536);
  memcpy(p + VIZ_FRAME_HEADER + 65536, viz_heat_write, 65536);
  memcpy(p + VIZ_FRAME_HEADER + 65536 * 2, viz_heat_ula, 65536);

  /* --- v1 appended data --- */

  /* Screen buffer (240*224 color indices) */
  if(oric->scr)
    memcpy(p + VIZ_FRAME_SIZE, oric->scr, VIZ_SCR_SIZE);
  else
    memset(p + VIZ_FRAME_SIZE, 0, VIZ_SCR_SIZE);

  /* vidbases[0..3] */
  for(i = 0; i < 4; i++)
    viz_write_u16_le(p + VIZ_FRAME_SIZE + VIZ_SCR_SIZE + i * 2, oric->vidbases[i]);

  /* Video RAM main area: vid_addr .. vid_addr+7999 (clamped to 64K) */
  vid_addr = oric->vid_addr;
  if(vid_addr + VIZ_VIDRAM_MAIN <= 65536)
    memcpy(p + VIZ_FRAME_SIZE + VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE,
           oric->mem + vid_addr, VIZ_VIDRAM_MAIN);
  else
  {
    int first = 65536 - vid_addr;
    memcpy(p + VIZ_FRAME_SIZE + VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE,
           oric->mem + vid_addr, first);
    memset(p + VIZ_FRAME_SIZE + VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE + first,
           0, VIZ_VIDRAM_MAIN - first);
  }

  /* Video RAM bottom 3 rows: vidbases[2] .. vidbases[2]+119 */
  vb2_addr = oric->vidbases[2];
  if(vb2_addr + VIZ_VIDRAM_BOTTOM <= 65536)
    memcpy(p + VIZ_FRAME_SIZE + VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE + VIZ_VIDRAM_MAIN,
           oric->mem + vb2_addr, VIZ_VIDRAM_BOTTOM);
  else
  {
    int first = 65536 - vb2_addr;
    memcpy(p + VIZ_FRAME_SIZE + VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE + VIZ_VIDRAM_MAIN,
           oric->mem + vb2_addr, first);
    memset(p + VIZ_FRAME_SIZE + VIZ_SCR_SIZE + VIZ_VIDBASES_SIZE + VIZ_VIDRAM_MAIN + first,
           0, VIZ_VIDRAM_BOTTOM - first);
  }

  viz.send_offset = 0;
  viz.frame_pending = SDL_TRUE;
  viz.frame_counter++;
}

/* Try to send as much of the pending frame as possible.
   Returns SDL_TRUE when the entire frame has been sent. */
static SDL_bool viz_flush_frame(void)
{
  while(viz.send_offset < VIZ_FRAME_SIZE_V1)
  {
    int n = send(viz.client_sock,
                 (const char *)(viz.frame_buf + viz.send_offset),
                 VIZ_FRAME_SIZE_V1 - viz.send_offset, 0);
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

  /* Allocate frame buffer */
  viz.frame_buf = (unsigned char *)malloc(VIZ_FRAME_SIZE_V1);
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

      /* Send an initial frame immediately so a paused emulator
         still shows its current screen to the newly connected client */
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

  /* When paused, don't build new frames — the last one is still valid */
  if(!running)
    return;

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
  int i;

  if(!viz_tracking)
    return;

  for(i = 0; i < 65536; i++)
  {
    if(viz_heat_read[i] > VIZ_DECAY_RATE)
      viz_heat_read[i] -= VIZ_DECAY_RATE;
    else
      viz_heat_read[i] = 0;

    if(viz_heat_write[i] > VIZ_DECAY_RATE)
      viz_heat_write[i] -= VIZ_DECAY_RATE;
    else
      viz_heat_write[i] = 0;

    if(viz_heat_ula[i] > VIZ_DECAY_RATE)
      viz_heat_ula[i] -= VIZ_DECAY_RATE;
    else
      viz_heat_ula[i] = 0;
  }
}
