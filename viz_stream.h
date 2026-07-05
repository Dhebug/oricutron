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
*/

#ifndef ORICUTRON_VIZ_STREAM_H
#define ORICUTRON_VIZ_STREAM_H

struct machine;

/* Initialize the viz streaming server on gdb_port + 1.
   Returns SDL_TRUE on success, SDL_FALSE on failure. */
SDL_bool viz_init(struct machine *oric);

/* Shut down the viz server, closing all sockets. */
void viz_shutdown(void);

/* Non-blocking poll: accept connections and push heatmap frames.
   Call from the main loop.  Pass running=SDL_TRUE when emulating,
   SDL_FALSE when paused (debug/menu) to freeze frame output. */
void viz_poll(struct machine *oric, SDL_bool running);

/* Decay heatmap values once per emulated frame. */
void viz_frame_decay(void);

/* Global enable flag - set to SDL_TRUE when server is listening */
extern SDL_bool viz_tracking;

/* Suppress flag - set to SDL_TRUE to ignore reads/writes
   (used by monitor and GDB stub to avoid polluting the heatmap) */
extern SDL_bool viz_suppress;

/* Heatmap arrays: 0-255 heat per address */
extern Uint8 viz_heat_read[65536];
extern Uint8 viz_heat_write[65536];
extern Uint8 viz_heat_ula[65536];

/* Inline marking functions */
#ifdef _MSC_VER
#define VIZ_INLINE static __inline
#else
#define VIZ_INLINE static inline
#endif

VIZ_INLINE void viz_mark_read(Uint16 addr)
{
  if(!viz_suppress) viz_heat_read[addr] = 255;
}

VIZ_INLINE void viz_mark_write(Uint16 addr)
{
  if(!viz_suppress) viz_heat_write[addr] = 255;
}

VIZ_INLINE void viz_mark_ula(Uint16 addr)
{
  viz_heat_ula[addr] = 255;
}

#endif /* ORICUTRON_VIZ_STREAM_H */
