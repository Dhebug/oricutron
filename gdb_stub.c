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
**  GDB Remote Serial Protocol stub implementation
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
#define GDB_INVALID_SOCKET INVALID_SOCKET
#define GDB_SOCKET_ERROR   SOCKET_ERROR
#define gdb_closesocket    closesocket
#define gdb_would_block()  (WSAGetLastError() == WSAEWOULDBLOCK)
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#define GDB_INVALID_SOCKET (-1)
#define GDB_SOCKET_ERROR   (-1)
#define gdb_closesocket    close
#define gdb_would_block()  (errno == EWOULDBLOCK || errno == EAGAIN)
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
#include "tape.h"
#include "gdb_stub.h"

extern char mon_bpmsg[];

/* ------------------------------------------------------------------ */
/* Internal state                                                      */
/* ------------------------------------------------------------------ */

#define GDB_BUF_SIZE  4096
#define GDB_PKT_SIZE  (GDB_BUF_SIZE * 2)

static struct {
  SOCKET listen_sock;
  SOCKET client_sock;
  SDL_bool initialized;
  SDL_bool wsa_initialized;
  int port;

  /* Receive buffer */
  char rxbuf[GDB_BUF_SIZE];
  int  rxlen;

  /* Packet assembly */
  char pktbuf[GDB_PKT_SIZE];

  /* Are we waiting for the client to see a stop reply? */
  SDL_bool stop_pending;

  /* Does the debugger UI need a re-render? */
  SDL_bool need_render;

  /* PC value at last stop notification sent to client */
  Uint16 last_notified_pc;
} gdb = {
  GDB_INVALID_SOCKET,
  GDB_INVALID_SOCKET,
  SDL_FALSE,
  SDL_FALSE,
  0,
  {0}, 0,
  {0},
  SDL_FALSE,
  SDL_FALSE,
  0
};

/* ------------------------------------------------------------------ */
/* Hex helpers                                                         */
/* ------------------------------------------------------------------ */

static const char hexchars[] = "0123456789abcdef";

static int hex_digit(char c)
{
  if(c >= '0' && c <= '9') return c - '0';
  if(c >= 'a' && c <= 'f') return c - 'a' + 10;
  if(c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static int hex_to_int(const char *p, int nchars, unsigned int *out)
{
  unsigned int val = 0;
  int i;
  for(i = 0; i < nchars; i++)
  {
    int d = hex_digit(p[i]);
    if(d < 0) return 0;
    val = (val << 4) | d;
  }
  *out = val;
  return 1;
}

/* Parse a hex string of variable length, stopped by delimiter or end.
   Returns number of chars consumed. */
static int hex_parse(const char *p, unsigned int *out)
{
  unsigned int val = 0;
  int count = 0;
  while(*p)
  {
    int d = hex_digit(*p);
    if(d < 0) break;
    val = (val << 4) | d;
    p++;
    count++;
  }
  *out = val;
  return count;
}

/* Write a byte as two hex chars */
static void hex_byte(char *dst, unsigned char val)
{
  dst[0] = hexchars[(val >> 4) & 0xf];
  dst[1] = hexchars[val & 0xf];
}

/* ------------------------------------------------------------------ */
/* Socket helpers                                                      */
/* ------------------------------------------------------------------ */

static SDL_bool gdb_socket_init(void)
{
#ifdef WIN32
  if(!gdb.wsa_initialized)
  {
    WSADATA wsadata;
    if(WSAStartup(MAKEWORD(2,2), &wsadata) != 0)
      return SDL_FALSE;
    gdb.wsa_initialized = SDL_TRUE;
  }
#endif
  return SDL_TRUE;
}

static void gdb_socket_cleanup(void)
{
#ifdef WIN32
  if(gdb.wsa_initialized)
  {
    WSACleanup();
    gdb.wsa_initialized = SDL_FALSE;
  }
#endif
}

static void gdb_set_nonblocking(SOCKET sock)
{
#ifdef WIN32
  u_long imode = 1;
  ioctlsocket(sock, FIONBIO, &imode);
#else
  fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
#endif
}

static void gdb_close_client(void)
{
  if(gdb.client_sock != GDB_INVALID_SOCKET)
  {
    gdb_closesocket(gdb.client_sock);
    gdb.client_sock = GDB_INVALID_SOCKET;
    gdb.rxlen = 0;
    gdb.stop_pending = SDL_FALSE;
    printf("GDB: client disconnected\n");
  }
}

/* ------------------------------------------------------------------ */
/* Packet send/receive                                                 */
/* ------------------------------------------------------------------ */

static void gdb_send_raw(const char *data, int len)
{
  if(gdb.client_sock == GDB_INVALID_SOCKET) return;

  while(len > 0)
  {
    int sent = send(gdb.client_sock, data, len, 0);
    if(sent <= 0)
    {
      gdb_close_client();
      return;
    }
    data += sent;
    len -= sent;
  }
}

static void gdb_send_ack(void)
{
  gdb_send_raw("+", 1);
}

static void gdb_send_packet(const char *data)
{
  unsigned char csum = 0;
  int i, len;
  char buf[GDB_PKT_SIZE + 5];

  len = (int)strlen(data);
  if(len > GDB_PKT_SIZE - 5) len = GDB_PKT_SIZE - 5;

  buf[0] = '$';
  for(i = 0; i < len; i++)
  {
    buf[i + 1] = data[i];
    csum += (unsigned char)data[i];
  }
  buf[len + 1] = '#';
  hex_byte(&buf[len + 2], csum);
  buf[len + 4] = '\0';

  gdb_send_raw(buf, len + 4);
}

static void gdb_send_empty(void)
{
  gdb_send_packet("");
}

static void gdb_send_ok(void)
{
  gdb_send_packet("OK");
}

static void gdb_send_error(int errnum)
{
  char buf[8];
  buf[0] = 'E';
  hex_byte(&buf[1], (unsigned char)errnum);
  buf[3] = '\0';
  gdb_send_packet(buf);
}

/* ------------------------------------------------------------------ */
/* Stop reply                                                          */
/* ------------------------------------------------------------------ */

static void gdb_send_stop_reply(struct machine *oric, int signal)
{
  char buf[64];
  struct m6502 *cpu = &oric->cpu;
  unsigned char flags = (unsigned char)MAKEFLAGS;

  /* T05 with register info for faster response */
  sprintf(buf, "T%02x00:%02x;01:%02x;02:%02x;03:%02x;04:%02x%02x;05:%02x;",
    signal,
    cpu->a,
    cpu->x,
    cpu->y,
    cpu->sp,
    (unsigned char)(cpu->pc & 0xff), (unsigned char)((cpu->pc >> 8) & 0xff),
    flags);
  gdb_send_packet(buf);
  gdb.last_notified_pc = cpu->pc;
}

/* ------------------------------------------------------------------ */
/* Register access                                                     */
/* ------------------------------------------------------------------ */

/* 6502 register order for GDB:
   0: A     (1 byte)
   1: X     (1 byte)
   2: Y     (1 byte)
   3: SP    (1 byte)
   4: PC    (2 bytes, little-endian)
   5: FLAGS (1 byte)
   Total: 7 bytes = 14 hex chars */

static void gdb_read_registers(struct machine *oric)
{
  char buf[32];
  struct m6502 *cpu = &oric->cpu;
  unsigned char flags = (unsigned char)MAKEFLAGS;
  char *p = buf;

  hex_byte(p, cpu->a);  p += 2;  /* reg 0: A */
  hex_byte(p, cpu->x);  p += 2;  /* reg 1: X */
  hex_byte(p, cpu->y);  p += 2;  /* reg 2: Y */
  hex_byte(p, cpu->sp); p += 2;  /* reg 3: SP */
  /* reg 4: PC (2 bytes, little-endian) */
  hex_byte(p, (unsigned char)(cpu->pc & 0xff));       p += 2;
  hex_byte(p, (unsigned char)((cpu->pc >> 8) & 0xff)); p += 2;
  hex_byte(p, flags);   p += 2;  /* reg 5: FLAGS */
  *p = '\0';

  gdb_send_packet(buf);
}

static void gdb_write_registers(struct machine *oric, const char *data)
{
  struct m6502 *cpu = &oric->cpu;
  unsigned int val;
  int len = (int)strlen(data);

  if(len < 14)
  {
    gdb_send_error(1);
    return;
  }

  hex_to_int(&data[0], 2, &val);  cpu->a = (Uint8)val;
  hex_to_int(&data[2], 2, &val);  cpu->x = (Uint8)val;
  hex_to_int(&data[4], 2, &val);  cpu->y = (Uint8)val;
  hex_to_int(&data[6], 2, &val);  cpu->sp = (Uint8)val;
  /* PC: little-endian, 2 bytes */
  {
    unsigned int lo, hi;
    hex_to_int(&data[8], 2, &lo);
    hex_to_int(&data[10], 2, &hi);
    cpu->pc = (Uint16)(lo | (hi << 8));
  }
  hex_to_int(&data[12], 2, &val);
  {
    Uint8 n = (Uint8)val;
    SETFLAGS(n);
  }

  gdb_send_ok();
}

static void gdb_read_register(struct machine *oric, const char *data)
{
  struct m6502 *cpu = &oric->cpu;
  unsigned int reg;
  char buf[8];
  unsigned char flags;

  hex_parse(data, &reg);

  switch(reg)
  {
    case 0: hex_byte(buf, cpu->a); buf[2] = '\0'; break;
    case 1: hex_byte(buf, cpu->x); buf[2] = '\0'; break;
    case 2: hex_byte(buf, cpu->y); buf[2] = '\0'; break;
    case 3: hex_byte(buf, cpu->sp); buf[2] = '\0'; break;
    case 4:
      hex_byte(&buf[0], (unsigned char)(cpu->pc & 0xff));
      hex_byte(&buf[2], (unsigned char)((cpu->pc >> 8) & 0xff));
      buf[4] = '\0';
      break;
    case 5:
      flags = (unsigned char)MAKEFLAGS;
      hex_byte(buf, flags);
      buf[2] = '\0';
      break;
    default:
      gdb_send_error(0);
      return;
  }
  gdb_send_packet(buf);
}

static void gdb_write_register(struct machine *oric, const char *data)
{
  struct m6502 *cpu = &oric->cpu;
  unsigned int reg, val;
  const char *eq;

  hex_parse(data, &reg);
  eq = strchr(data, '=');
  if(!eq)
  {
    gdb_send_error(1);
    return;
  }
  eq++;

  switch(reg)
  {
    case 0: hex_to_int(eq, 2, &val); cpu->a = (Uint8)val; break;
    case 1: hex_to_int(eq, 2, &val); cpu->x = (Uint8)val; break;
    case 2: hex_to_int(eq, 2, &val); cpu->y = (Uint8)val; break;
    case 3: hex_to_int(eq, 2, &val); cpu->sp = (Uint8)val; break;
    case 4:
    {
      unsigned int lo, hi;
      hex_to_int(eq, 2, &lo);
      hex_to_int(eq + 2, 2, &hi);
      cpu->pc = (Uint16)(lo | (hi << 8));
      break;
    }
    case 5:
    {
      hex_to_int(eq, 2, &val);
      {
        Uint8 n = (Uint8)val;
        SETFLAGS(n);
      }
      break;
    }
    default:
      gdb_send_error(0);
      return;
  }
  gdb_send_ok();
}

/* ------------------------------------------------------------------ */
/* Memory access                                                       */
/* ------------------------------------------------------------------ */

static void gdb_read_memory(struct machine *oric, const char *data)
{
  unsigned int addr, len;
  unsigned int i;
  const char *comma;
  char *p;

  hex_parse(data, &addr);
  comma = strchr(data, ',');
  if(!comma)
  {
    gdb_send_error(1);
    return;
  }
  hex_parse(comma + 1, &len);

  if(len > (GDB_PKT_SIZE - 5) / 2)
    len = (GDB_PKT_SIZE - 5) / 2;

  p = gdb.pktbuf;
  for(i = 0; i < len; i++)
  {
    unsigned char byte = oric->cpu.read(&oric->cpu, (Uint16)((addr + i) & 0xffff));
    hex_byte(p, byte);
    p += 2;
  }
  *p = '\0';

  gdb_send_packet(gdb.pktbuf);
}

static void gdb_write_memory(struct machine *oric, const char *data)
{
  unsigned int addr, len;
  unsigned int i;
  const char *comma, *colon;

  hex_parse(data, &addr);
  comma = strchr(data, ',');
  if(!comma)
  {
    gdb_send_error(1);
    return;
  }
  hex_parse(comma + 1, &len);
  colon = strchr(data, ':');
  if(!colon)
  {
    gdb_send_error(1);
    return;
  }
  colon++;

  for(i = 0; i < len; i++)
  {
    unsigned int val;
    if(!hex_to_int(colon + i * 2, 2, &val))
    {
      gdb_send_error(1);
      return;
    }
    oric->cpu.write(&oric->cpu, (Uint16)((addr + i) & 0xffff), (Uint8)val);
  }

  gdb_send_ok();
}

/* ------------------------------------------------------------------ */
/* Breakpoints / Watchpoints                                           */
/* ------------------------------------------------------------------ */

static void gdb_set_breakpoint(struct machine *oric, const char *data)
{
  unsigned int addr;
  int i;
  const char *comma;

  comma = strchr(data, ',');
  if(!comma)
  {
    gdb_send_error(1);
    return;
  }
  hex_parse(comma + 1, &addr);

  /* Find empty breakpoint slot */
  for(i = 0; i < 16; i++)
  {
    if(oric->cpu.breakpoints[i] == -1)
    {
      oric->cpu.breakpoints[i] = (Sint32)addr;
      oric->cpu.anybp = SDL_TRUE;
      gdb_send_ok();
      return;
    }
  }

  /* All slots full */
  gdb_send_error(0x20);
}

static void gdb_clear_breakpoint(struct machine *oric, const char *data)
{
  unsigned int addr;
  int i;
  SDL_bool any = SDL_FALSE;
  const char *comma;

  comma = strchr(data, ',');
  if(!comma)
  {
    gdb_send_error(1);
    return;
  }
  hex_parse(comma + 1, &addr);

  for(i = 0; i < 16; i++)
  {
    if(oric->cpu.breakpoints[i] == (Sint32)addr)
    {
      oric->cpu.breakpoints[i] = -1;
      break;
    }
  }

  /* Recompute anybp */
  for(i = 0; i < 16; i++)
  {
    if(oric->cpu.breakpoints[i] != -1)
    {
      any = SDL_TRUE;
      break;
    }
  }
  oric->cpu.anybp = any;

  gdb_send_ok();
}

static void gdb_set_watchpoint(struct machine *oric, const char *data, Uint8 flags)
{
  unsigned int addr;
  int i;
  const char *comma;

  comma = strchr(data, ',');
  if(!comma)
  {
    gdb_send_error(1);
    return;
  }
  hex_parse(comma + 1, &addr);

  /* Find empty watchpoint slot */
  for(i = 0; i < 16; i++)
  {
    if(oric->cpu.membreakpoints[i].flags == 0)
    {
      oric->cpu.membreakpoints[i].addr = (Uint16)addr;
      oric->cpu.membreakpoints[i].flags = flags;
      oric->cpu.membreakpoints[i].lastval = oric->cpu.read(&oric->cpu, (Uint16)addr);
      oric->cpu.anymbp = SDL_TRUE;
      gdb_send_ok();
      return;
    }
  }

  gdb_send_error(0x20);
}

static void gdb_clear_watchpoint(struct machine *oric, const char *data, Uint8 flags)
{
  unsigned int addr;
  int i;
  SDL_bool any = SDL_FALSE;
  const char *comma;

  (void)flags;

  comma = strchr(data, ',');
  if(!comma)
  {
    gdb_send_error(1);
    return;
  }
  hex_parse(comma + 1, &addr);

  for(i = 0; i < 16; i++)
  {
    if(oric->cpu.membreakpoints[i].addr == (Uint16)addr &&
       oric->cpu.membreakpoints[i].flags != 0)
    {
      oric->cpu.membreakpoints[i].flags = 0;
      break;
    }
  }

  /* Recompute anymbp */
  for(i = 0; i < 16; i++)
  {
    if(oric->cpu.membreakpoints[i].flags != 0)
    {
      any = SDL_TRUE;
      break;
    }
  }
  oric->cpu.anymbp = any;

  gdb_send_ok();
}

/* ------------------------------------------------------------------ */
/* Single step (mirrors steppy_step from monitor.c)                    */
/* ------------------------------------------------------------------ */

static void gdb_single_step(struct machine *oric)
{
  m6502_set_icycles(&oric->cpu, SDL_FALSE, mon_bpmsg);
  tape_patches(oric);
  via_clock(&oric->via, oric->cpu.icycles);
  ay_ticktock(&oric->ay, oric->cpu.icycles);

  switch(oric->drivetype)
  {
    case DRV_MICRODISC:
    case DRV_BD500:
    case DRV_JASMIN:
      wd17xx_ticktock(&oric->wddisk, oric->cpu.icycles);
      break;
  }

  if(oric->type == MACH_TELESTRAT)
    via_clock(&oric->tele_via, oric->cpu.icycles);

  if(oric->aciabackend)
    acia_clock(&oric->tele_acia, oric->cpu.icycles);

  oric->cpu.rastercycles -= oric->cpu.icycles;
  m6502_inst(&oric->cpu);

  if(oric->cpu.rastercycles <= 0)
  {
    ula_doraster(oric);
    oric->cpu.rastercycles += oric->cyclesperraster;
  }

  mon_bpmsg[0] = 0;

  /* Send stop reply with SIGTRAP */
  gdb_send_stop_reply(oric, GDB_SIGNAL_TRAP);
}

/* ------------------------------------------------------------------ */
/* Query handlers                                                      */
/* ------------------------------------------------------------------ */

static void gdb_handle_query(struct machine *oric, const char *data)
{
  (void)oric;

  if(strncmp(data, "Supported", 9) == 0)
  {
    gdb_send_packet("PacketSize=1000;swbreak+;hwbreak+;qRcmd+");
    return;
  }

  if(strcmp(data, "Attached") == 0)
  {
    gdb_send_packet("1");
    return;
  }

  if(strncmp(data, "Xfer", 4) == 0)
  {
    gdb_send_empty();
    return;
  }

  if(strcmp(data, "fThreadInfo") == 0)
  {
    gdb_send_packet("m1");
    return;
  }

  if(strcmp(data, "sThreadInfo") == 0)
  {
    gdb_send_packet("l");
    return;
  }

  if(strcmp(data, "C") == 0)
  {
    /* Current thread */
    gdb_send_packet("QC1");
    return;
  }

  if(strcmp(data, "OricCpuExtra") == 0)
  {
    /* Return extra CPU/machine debug state:
       L:<lastpc>;C:<cycles 4 bytes>;F:<frames 4 bytes>;R:<raster 2 bytes>;
       N:<NMI vec>;T:<RST vec>;I:<IRQ vec> */
    char buf[80];
    char *p = buf;
    Uint16 nmi, rst, irq;

    *p++ = 'L'; *p++ = ':';
    hex_byte(p, oric->cpu.lastpc & 0xff);       p += 2;
    hex_byte(p, (oric->cpu.lastpc >> 8) & 0xff); p += 2;
    *p++ = ';';

    *p++ = 'C'; *p++ = ':';
    hex_byte(p, oric->cpu.cycles & 0xff);         p += 2;
    hex_byte(p, (oric->cpu.cycles >> 8) & 0xff);  p += 2;
    hex_byte(p, (oric->cpu.cycles >> 16) & 0xff); p += 2;
    hex_byte(p, (oric->cpu.cycles >> 24) & 0xff); p += 2;
    *p++ = ';';

    *p++ = 'F'; *p++ = ':';
    hex_byte(p, oric->frames & 0xff);         p += 2;
    hex_byte(p, (oric->frames >> 8) & 0xff);  p += 2;
    hex_byte(p, (oric->frames >> 16) & 0xff); p += 2;
    hex_byte(p, (oric->frames >> 24) & 0xff); p += 2;
    *p++ = ';';

    *p++ = 'R'; *p++ = ':';
    hex_byte(p, oric->vid_raster & 0xff);       p += 2;
    hex_byte(p, (oric->vid_raster >> 8) & 0xff); p += 2;
    *p++ = ';';

    nmi = (oric->cpu.read(&oric->cpu, 0xfffb) << 8) | oric->cpu.read(&oric->cpu, 0xfffa);
    rst = (oric->cpu.read(&oric->cpu, 0xfffd) << 8) | oric->cpu.read(&oric->cpu, 0xfffc);
    irq = (oric->cpu.read(&oric->cpu, 0xffff) << 8) | oric->cpu.read(&oric->cpu, 0xfffe);

    *p++ = 'N'; *p++ = ':';
    hex_byte(p, nmi & 0xff);       p += 2;
    hex_byte(p, (nmi >> 8) & 0xff); p += 2;
    *p++ = ';';

    *p++ = 'T'; *p++ = ':';
    hex_byte(p, rst & 0xff);       p += 2;
    hex_byte(p, (rst >> 8) & 0xff); p += 2;
    *p++ = ';';

    *p++ = 'I'; *p++ = ':';
    hex_byte(p, irq & 0xff);       p += 2;
    hex_byte(p, (irq >> 8) & 0xff); p += 2;
    *p = '\0';

    gdb_send_packet(buf);
    return;
  }

  if(strcmp(data, "OricPeripherals") == 0)
  {
    /* Return internal peripheral state without side effects:
       V:<32 hex VIA regs>;A:<30 hex AY regs>;F:<8 hex WD17xx>;M:<12 hex Microdisc+drive> */
    char buf[128];
    char *p = buf;
    int i;

    /* VIA registers (side-effect-free monitor read) */
    *p++ = 'V'; *p++ = ':';
    for(i = 0; i < 16; i++)
    {
      hex_byte(p, via_mon_read(&oric->via, i));
      p += 2;
    }
    *p++ = ';';

    /* AY-3-8912 registers (direct read from internal state) */
    *p++ = 'A'; *p++ = ':';
    for(i = 0; i < NUM_AY_REGS; i++)
    {
      hex_byte(p, oric->ay.eregs[i]);
      p += 2;
    }
    *p++ = ';';

    /* WD17xx FDC registers */
    *p++ = 'F'; *p++ = ':';
    hex_byte(p, oric->wddisk.r_status);  p += 2;
    hex_byte(p, oric->wddisk.r_track);   p += 2;
    hex_byte(p, oric->wddisk.r_sector);  p += 2;
    hex_byte(p, oric->wddisk.r_data);    p += 2;
    *p++ = ';';

    /* Microdisc state */
    *p++ = 'M'; *p++ = ':';
    hex_byte(p, oric->md.status);         p += 2;
    hex_byte(p, oric->md.intrq);          p += 2;
    hex_byte(p, oric->md.drq);            p += 2;
    hex_byte(p, oric->wddisk.c_drive);    p += 2;
    hex_byte(p, oric->wddisk.c_side);     p += 2;
    hex_byte(p, oric->wddisk.c_track);    p += 2;
    *p++ = ';';

    /* ACIA 6551 registers */
    *p++ = 'C'; *p++ = ':';
    for(i = 0; i < ACIA_LAST; i++)
    {
      hex_byte(p, oric->tele_acia.regs[i]);
      p += 2;
    }
    *p = '\0';

    gdb_send_packet(buf);
    return;
  }

  /* qOricCmd - Execute monitor command (hex-encoded), return hex-encoded output */
  if(strncmp(data, "OricCmd,", 8) == 0)
  {
    const char *hex = data + 8;
    int hexlen = (int)strlen(hex);
    int cmdlen = hexlen / 2;
    char *cmd;
    char *output;
    SDL_bool needrender = SDL_FALSE;
    int i, olen;

    if(cmdlen <= 0) { gdb_send_empty(); return; }

    cmd = (char *)malloc(cmdlen + 1);
    if(!cmd) { gdb_send_packet("E01"); return; }
    for(i = 0; i < cmdlen; i++)
      cmd[i] = (char)((hex_digit(hex[i*2]) << 4) | hex_digit(hex[i*2+1]));
    cmd[cmdlen] = 0;

    mon_capture_start(GDB_PKT_SIZE / 2);
    mon_cmd(cmd, oric, &needrender);
    output = mon_capture_end();
    free(cmd);

    if(needrender) gdb.need_render = SDL_TRUE;

    if(output && output[0])
    {
      char *reply;
      olen = (int)strlen(output);
      reply = (char *)malloc(olen * 2 + 1);
      if(reply)
      {
        for(i = 0; i < olen; i++)
          hex_byte(reply + i * 2, (unsigned char)output[i]);
        reply[olen * 2] = 0;
        gdb_send_packet(reply);
        free(reply);
      }
      else
      {
        gdb_send_empty();
      }
    }
    else
    {
      gdb_send_empty();
    }
    if(output) free(output);
    return;
  }

  /* qOricEval - Evaluate expression, return hex result (plain text, no encoding) */
  if(strncmp(data, "OricEval,", 9) == 0)
  {
    const char *hex = data + 9;
    int hexlen = (int)strlen(hex);
    int exprlen = hexlen / 2;
    char *expr;
    unsigned int result;
    int i, off;

    if(exprlen <= 0) { gdb_send_packet("E01"); return; }

    expr = (char *)malloc(exprlen + 1);
    if(!expr) { gdb_send_packet("E01"); return; }
    for(i = 0; i < exprlen; i++)
      expr[i] = (char)((hex_digit(hex[i*2]) << 4) | hex_digit(hex[i*2+1]));
    expr[exprlen] = 0;

    off = 0;
    if(mon_eval(oric, &result, expr, &off))
    {
      char buf[16];
      snprintf(buf, sizeof(buf), "V%04X", result);  /* V prefix = value, avoids clash with Enn errors */
      gdb_send_packet(buf);
    }
    else
    {
      gdb_send_packet("E02");
    }
    free(expr);
    return;
  }

  /* Unknown query - empty response */
  gdb_send_empty();
}

/* ------------------------------------------------------------------ */
/* Command dispatch                                                    */
/* ------------------------------------------------------------------ */

static void gdb_handle_packet(struct machine *oric, const char *data, int len)
{
  (void)len;

  switch(data[0])
  {
    case '?':
      /* Halt reason */
      gdb_send_stop_reply(oric, GDB_SIGNAL_TRAP);
      break;

    case 'g':
      /* Read all registers */
      gdb_read_registers(oric);
      break;

    case 'G':
      /* Write all registers */
      gdb_write_registers(oric, data + 1);
      gdb.need_render = SDL_TRUE;
      break;

    case 'p':
      /* Read single register */
      gdb_read_register(oric, data + 1);
      break;

    case 'P':
      /* Write single register */
      gdb_write_register(oric, data + 1);
      gdb.need_render = SDL_TRUE;
      break;

    case 'm':
      /* Read memory */
      gdb_read_memory(oric, data + 1);
      break;

    case 'M':
      /* Write memory */
      gdb_write_memory(oric, data + 1);
      gdb.need_render = SDL_TRUE;
      break;

    case 'c':
      /* Continue */
      if(data[1])
      {
        /* Continue at address */
        unsigned int addr;
        hex_parse(data + 1, &addr);
        oric->cpu.pc = (Uint16)addr;
      }
      setemumode(oric, NULL, EM_RUNNING);
      gdb.stop_pending = SDL_TRUE;
      break;

    case 's':
      /* Single step */
      if(data[1])
      {
        /* Step from address */
        unsigned int addr;
        hex_parse(data + 1, &addr);
        oric->cpu.pc = (Uint16)addr;
      }
      gdb_single_step(oric);
      gdb.need_render = SDL_TRUE;
      break;

    case 'Z':
      /* Set breakpoint/watchpoint */
      switch(data[1])
      {
        case '0': /* software breakpoint */
        case '1': /* hardware breakpoint */
          gdb_set_breakpoint(oric, data + 1);
          break;
        case '2': /* write watchpoint */
          gdb_set_watchpoint(oric, data + 1, MBPF_WRITE);
          break;
        case '3': /* read watchpoint */
          gdb_set_watchpoint(oric, data + 1, MBPF_READ);
          break;
        case '4': /* access watchpoint */
          gdb_set_watchpoint(oric, data + 1, MBPF_READ | MBPF_WRITE);
          break;
        default:
          gdb_send_empty();
          break;
      }
      break;

    case 'z':
      /* Clear breakpoint/watchpoint */
      switch(data[1])
      {
        case '0':
        case '1':
          gdb_clear_breakpoint(oric, data + 1);
          break;
        case '2':
          gdb_clear_watchpoint(oric, data + 1, MBPF_WRITE);
          break;
        case '3':
          gdb_clear_watchpoint(oric, data + 1, MBPF_READ);
          break;
        case '4':
          gdb_clear_watchpoint(oric, data + 1, MBPF_READ | MBPF_WRITE);
          break;
        default:
          gdb_send_empty();
          break;
      }
      break;

    case 'D':
      /* Detach */
      gdb_send_ok();
      gdb_close_client();
      setemumode(oric, NULL, EM_RUNNING);
      break;

    case 'k':
      /* Kill - close client but keep emulator running */
      gdb_close_client();
      setemumode(oric, NULL, EM_RUNNING);
      break;

    case 'q':
      /* Query */
      gdb_handle_query(oric, data + 1);
      break;

    case 'H':
      /* Set thread - we only have one thread */
      gdb_send_ok();
      break;

    case 'v':
      if(strncmp(data, "vMustReplyEmpty", 15) == 0)
        gdb_send_empty();
      else if(strncmp(data, "vCont?", 6) == 0)
        gdb_send_packet("vCont;c;s");
      else if(strncmp(data, "vCont;c", 7) == 0)
      {
        setemumode(oric, NULL, EM_RUNNING);
        gdb.stop_pending = SDL_TRUE;
      }
      else if(strncmp(data, "vCont;s", 7) == 0)
      {
        gdb_single_step(oric);
        gdb.need_render = SDL_TRUE;
      }
      else
        gdb_send_empty();
      break;

    default:
      /* Unknown command */
      gdb_send_empty();
      break;
  }
}

/* ------------------------------------------------------------------ */
/* Packet parser                                                       */
/* ------------------------------------------------------------------ */

/* Process received data, extract and dispatch complete packets.
   Returns SDL_TRUE if a command caused the emulator to resume (continue). */
static SDL_bool gdb_process_data(struct machine *oric)
{
  int i;
  SDL_bool resumed = SDL_FALSE;

  while(gdb.rxlen > 0)
  {
    /* Check for 0x03 (Ctrl-C / break) */
    if(gdb.rxbuf[0] == 0x03)
    {
      /* Remove the byte */
      gdb.rxlen--;
      memmove(gdb.rxbuf, gdb.rxbuf + 1, gdb.rxlen);

      /* Interrupt: pause emulation */
      if(oric->emu_mode == EM_RUNNING)
      {
        setemumode(oric, NULL, EM_DEBUG);
        gdb.stop_pending = SDL_FALSE;
        gdb.need_render = SDL_TRUE;
        gdb_send_stop_reply(oric, GDB_SIGNAL_INT);
      }
      continue;
    }

    /* Skip ACK/NAK */
    if(gdb.rxbuf[0] == '+' || gdb.rxbuf[0] == '-')
    {
      gdb.rxlen--;
      memmove(gdb.rxbuf, gdb.rxbuf + 1, gdb.rxlen);
      continue;
    }

    /* Look for packet start */
    if(gdb.rxbuf[0] != '$')
    {
      /* Skip unknown byte */
      gdb.rxlen--;
      memmove(gdb.rxbuf, gdb.rxbuf + 1, gdb.rxlen);
      continue;
    }

    /* Find packet end '#' */
    for(i = 1; i < gdb.rxlen; i++)
    {
      if(gdb.rxbuf[i] == '#')
        break;
    }

    /* Need '#' plus 2 checksum chars */
    if(i >= gdb.rxlen || (i + 2) >= gdb.rxlen)
      break; /* incomplete packet, wait for more data */

    /* Extract packet data (between $ and #) */
    {
      int pkt_len = i - 1;
      int total_len = i + 3; /* $...#xx */
      unsigned char csum_calc = 0;
      unsigned int csum_recv;
      int j;

      for(j = 1; j <= pkt_len; j++)
        csum_calc += (unsigned char)gdb.rxbuf[j];

      hex_to_int(&gdb.rxbuf[i + 1], 2, &csum_recv);

      if(csum_calc == (unsigned char)csum_recv)
      {
        /* Valid packet */
        gdb_send_ack();

        /* Copy packet data to pktbuf for processing */
        if(pkt_len < GDB_PKT_SIZE - 1)
        {
          int prev_mode = oric->emu_mode;
          memcpy(gdb.pktbuf, &gdb.rxbuf[1], pkt_len);
          gdb.pktbuf[pkt_len] = '\0';
          gdb_handle_packet(oric, gdb.pktbuf, pkt_len);
          if(prev_mode != EM_RUNNING && oric->emu_mode == EM_RUNNING)
            resumed = SDL_TRUE;
        }
      }
      else
      {
        /* Bad checksum - request retransmit */
        gdb_send_raw("-", 1);
      }

      /* Remove processed data from buffer */
      gdb.rxlen -= total_len;
      if(gdb.rxlen > 0)
        memmove(gdb.rxbuf, gdb.rxbuf + total_len, gdb.rxlen);
    }
  }

  return resumed;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

SDL_bool gdb_stub_init(struct machine *oric, int port)
{
  struct sockaddr_in addr;
  int on = 1;

  (void)oric;

  if(gdb.initialized)
    return SDL_TRUE;

  if(!gdb_socket_init())
  {
    printf("GDB: failed to initialize sockets\n");
    return SDL_FALSE;
  }

  gdb.listen_sock = socket(AF_INET, SOCK_STREAM, 0);
  if(gdb.listen_sock == GDB_INVALID_SOCKET)
  {
    printf("GDB: failed to create socket\n");
    return SDL_FALSE;
  }

  setsockopt(gdb.listen_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&on, sizeof(on));
  gdb_set_nonblocking(gdb.listen_sock);

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons((unsigned short)port);

  if(bind(gdb.listen_sock, (struct sockaddr *)&addr, sizeof(addr)) == GDB_SOCKET_ERROR)
  {
    printf("GDB: failed to bind to port %d\n", port);
    gdb_closesocket(gdb.listen_sock);
    gdb.listen_sock = GDB_INVALID_SOCKET;
    return SDL_FALSE;
  }

  if(listen(gdb.listen_sock, 1) == GDB_SOCKET_ERROR)
  {
    printf("GDB: failed to listen on port %d\n", port);
    gdb_closesocket(gdb.listen_sock);
    gdb.listen_sock = GDB_INVALID_SOCKET;
    return SDL_FALSE;
  }

  gdb.port = port;
  gdb.initialized = SDL_TRUE;
  gdb.client_sock = GDB_INVALID_SOCKET;
  gdb.rxlen = 0;
  gdb.stop_pending = SDL_FALSE;

  printf("GDB: server listening on port %d\n", port);
  return SDL_TRUE;
}

void gdb_stub_shutdown(void)
{
  if(!gdb.initialized)
    return;

  gdb_close_client();

  if(gdb.listen_sock != GDB_INVALID_SOCKET)
  {
    gdb_closesocket(gdb.listen_sock);
    gdb.listen_sock = GDB_INVALID_SOCKET;
  }

  gdb_socket_cleanup();
  gdb.initialized = SDL_FALSE;

  printf("GDB: server shut down\n");
}

SDL_bool gdb_stub_poll(struct machine *oric)
{
  SDL_bool render;

  if(!gdb.initialized)
    return SDL_FALSE;

  /* Accept new connections */
  if(gdb.client_sock == GDB_INVALID_SOCKET)
  {
    struct sockaddr_in client_addr;
    socklen_t addrlen = sizeof(client_addr);
    SOCKET newsock = accept(gdb.listen_sock, (struct sockaddr *)&client_addr, &addrlen);
    if(newsock != GDB_INVALID_SOCKET)
    {
      gdb.client_sock = newsock;
      gdb_set_nonblocking(gdb.client_sock);
      gdb.rxlen = 0;
      gdb.stop_pending = SDL_FALSE;
      gdb.need_render = SDL_FALSE;
      gdb.last_notified_pc = oric->cpu.pc;
      printf("GDB: client connected\n");

      /* Pause emulation when client connects */
      if(oric->emu_mode == EM_RUNNING)
        setemumode(oric, NULL, EM_DEBUG);

      return SDL_TRUE;
    }
    return SDL_FALSE;
  }

  /* Read available data from client */
  {
    int space = GDB_BUF_SIZE - gdb.rxlen;
    if(space > 0)
    {
      int n = recv(gdb.client_sock, gdb.rxbuf + gdb.rxlen, space, 0);
      if(n > 0)
      {
        gdb.rxlen += n;
      }
      else if(n == 0)
      {
        /* Client disconnected */
        gdb_close_client();
        return SDL_FALSE;
      }
      else
      {
        /* n < 0: check if it's a real error or just WOULDBLOCK */
        if(!gdb_would_block())
        {
          gdb_close_client();
          return SDL_FALSE;
        }
      }
    }
  }

  /* Process any complete packets */
  if(gdb.rxlen > 0)
    gdb_process_data(oric);

  /* Detect state changes not initiated by GDB (e.g. user stepping in
     Oricutron's built-in monitor).  If the emulator is stopped and PC
     differs from the last value we reported, send a fresh stop reply
     so the debug client stays in sync. */
  if(gdb.client_sock != GDB_INVALID_SOCKET &&
     oric->emu_mode != EM_RUNNING &&
     oric->cpu.pc != gdb.last_notified_pc)
  {
    gdb_send_stop_reply(oric, GDB_SIGNAL_TRAP);
    gdb.need_render = SDL_TRUE;
  }

  /* Return and reset render flag */
  render = gdb.need_render;
  gdb.need_render = SDL_FALSE;
  return render;
}

void gdb_stub_notify_stop(struct machine *oric, int reason)
{
  if(!gdb.initialized || gdb.client_sock == GDB_INVALID_SOCKET)
    return;

  if(!gdb.stop_pending)
    return;

  gdb.stop_pending = SDL_FALSE;
  gdb_send_stop_reply(oric, reason);
}

SDL_bool gdb_stub_is_connected(void)
{
  return (gdb.initialized && gdb.client_sock != GDB_INVALID_SOCKET) ? SDL_TRUE : SDL_FALSE;
}

SDL_bool gdb_stub_is_listening(void)
{
  return gdb.initialized;
}
