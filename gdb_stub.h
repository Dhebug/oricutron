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
**  GDB Remote Serial Protocol stub header
*/

#ifndef ORICUTRON_GDB_STUB_H
#define ORICUTRON_GDB_STUB_H

struct machine;

/* Initialize GDB RSP server on the given TCP port.
   Returns SDL_TRUE on success, SDL_FALSE on failure. */
SDL_bool gdb_stub_init(struct machine *oric, int port);

/* Shut down the GDB RSP server, closing all sockets. */
void gdb_stub_shutdown(void);

/* Non-blocking poll for incoming GDB commands.
   Call from the main loop (both EM_RUNNING and EM_DEBUG paths).
   Returns SDL_TRUE if the debugger UI should be re-rendered. */
SDL_bool gdb_stub_poll(struct machine *oric);

/* Notify the GDB client that emulation has stopped (breakpoint, step, etc.).
   reason: 5 = SIGTRAP (breakpoint/step), 2 = SIGINT (interrupt) */
void gdb_stub_notify_stop(struct machine *oric, int reason);

/* Returns SDL_TRUE if a GDB client is currently connected. */
SDL_bool gdb_stub_is_connected(void);

/* Returns SDL_TRUE if the GDB server is initialized (listening for connections). */
SDL_bool gdb_stub_is_listening(void);

/* Reset transient stub state after the machine is reinitialized under an attached
   client (machine-type switch): drops a dangling temp breakpoint and pending-stop
   so the stub can't hang. Safe to call unconditionally; no-op if nothing pending. */
void gdb_stub_machine_reset(struct machine *oric);

/* Stop reason codes */
#define GDB_SIGNAL_INT   2   /* SIGINT - interrupt */
#define GDB_SIGNAL_TRAP  5   /* SIGTRAP - breakpoint/step */

#endif /* ORICUTRON_GDB_STUB_H */
