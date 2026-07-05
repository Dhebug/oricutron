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
  14      2        Reserved (0)
  16      65536    Read heatmap
  65552   65536    Write heatmap
  131088  65536    ULA heatmap
  Total: 196624 bytes
*/

#define VIZ_FRAME_HEADER  16
#define VIZ_FRAME_SIZE    (VIZ_FRAME_HEADER + 65536 * 3)
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
  unsigned char *frame_buf;   /* VIZ_FRAME_SIZE bytes, allocated once */
  int send_offset;            /* Resume offset for partial frame sends */
  SDL_bool frame_pending;     /* True if a partially-sent frame is in progress */
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

  /* Reserved */
  p[14] = 0;
  p[15] = 0;

  /* Copy heatmap arrays */
  memcpy(p + VIZ_FRAME_HEADER, viz_heat_read, 65536);
  memcpy(p + VIZ_FRAME_HEADER + 65536, viz_heat_write, 65536);
  memcpy(p + VIZ_FRAME_HEADER + 65536 * 2, viz_heat_ula, 65536);

  viz.send_offset = 0;
  viz.frame_pending = SDL_TRUE;
  viz.frame_counter++;
}

/* Try to send as much of the pending frame as possible.
   Returns SDL_TRUE when the entire frame has been sent. */
static SDL_bool viz_flush_frame(void)
{
  while(viz.send_offset < VIZ_FRAME_SIZE)
  {
    int n = send(viz.client_sock,
                 (const char *)(viz.frame_buf + viz.send_offset),
                 VIZ_FRAME_SIZE - viz.send_offset, 0);
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
  viz.frame_buf = (unsigned char *)malloc(VIZ_FRAME_SIZE);
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
    }
    return;
  }

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

  /* Check if client is still alive by peeking */
  {
    char peek;
    int n = recv(viz.client_sock, &peek, 1, MSG_PEEK);
    if(n == 0)
    {
      viz_close_client();
      return;
    }
    else if(n < 0 && !viz_would_block())
    {
      viz_close_client();
      return;
    }
  }

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
