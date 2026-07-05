#!/usr/bin/env python3
"""Test Oricutron GDB stub: qOricCmd and qOricEval commands."""

import socket
import sys
import time

GDB_HOST = '127.0.0.1'
GDB_PORT = 6503

def gdb_checksum(data):
    return sum(ord(c) for c in data) & 0xFF

def gdb_send(sock, data):
    """Send a GDB packet and return the response payload."""
    pkt = f'${data}#{gdb_checksum(data):02x}'
    sock.sendall(pkt.encode('ascii'))
    # Read response
    buf = b''
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            break
        buf += chunk
        # Look for complete packet: +$...#xx
        decoded = buf.decode('ascii', errors='replace')
        if '#' in decoded:
            # Find the packet payload
            start = decoded.find('$')
            end = decoded.find('#', start)
            if start >= 0 and end >= 0 and len(decoded) >= end + 3:
                return decoded[start+1:end]
    return None

def hex_encode(s):
    return ''.join(f'{ord(c):02x}' for c in s)

def hex_decode(h):
    out = ''
    for i in range(0, len(h), 2):
        out += chr(int(h[i:i+2], 16))
    return out

def oric_eval(sock, expr):
    """Evaluate expression via qOricEval, return (success, value_or_error).
    Protocol: success = 'VHHHH' (V prefix + hex value), error = 'Enn'."""
    resp = gdb_send(sock, f'qOricEval,{hex_encode(expr)}')
    if resp is None:
        return False, 'No response'
    if resp.startswith('V'):
        try:
            return True, int(resp[1:], 16)
        except ValueError:
            return False, f'Bad value response: {resp}'
    if resp.startswith('E'):
        return False, f'Error code {resp}'
    return False, f'Unexpected response: {resp}'

def oric_cmd(sock, cmd):
    """Execute monitor command via qOricCmd, return output text."""
    resp = gdb_send(sock, f'qOricCmd,{hex_encode(cmd)}')
    if resp is None:
        return None
    return hex_decode(resp)

# ---- Test helpers ----
passed = 0
failed = 0
errors = []

def test_eval(sock, expr, expected, desc=None):
    global passed, failed, errors
    label = desc or f'= {expr}'
    ok, val = oric_eval(sock, expr)
    if not ok:
        failed += 1
        errors.append(f'FAIL [{label}]: error - {val}')
        print(f'  FAIL [{label}]: error - {val}')
        return
    if val != expected:
        failed += 1
        errors.append(f'FAIL [{label}]: expected {expected} (${expected:04X}), got {val} (${val:04X})')
        print(f'  FAIL [{label}]: expected {expected} (${expected:04X}), got {val} (${val:04X})')
    else:
        passed += 1
        print(f'  OK   [{label}]: ${val:04X} ({val})')

def test_eval_fail(sock, expr, desc=None):
    global passed, failed, errors
    label = desc or f'= {expr} (should fail)'
    ok, val = oric_eval(sock, expr)
    if ok:
        failed += 1
        errors.append(f'FAIL [{label}]: expected error, got {val} (${val:04X})')
        print(f'  FAIL [{label}]: expected error, got {val} (${val:04X})')
    else:
        passed += 1
        print(f'  OK   [{label}]: correctly returned error')

def test_cmd(sock, cmd, expect_contains=None, expect_not_contains=None, desc=None):
    global passed, failed, errors
    label = desc or f'! {cmd}'
    output = oric_cmd(sock, cmd)
    if output is None:
        failed += 1
        errors.append(f'FAIL [{label}]: no response')
        print(f'  FAIL [{label}]: no response')
        return output

    ok = True
    if expect_contains:
        for s in (expect_contains if isinstance(expect_contains, list) else [expect_contains]):
            if s not in output:
                ok = False
                failed += 1
                errors.append(f'FAIL [{label}]: expected "{s}" in output, got: {output!r}')
                print(f'  FAIL [{label}]: expected "{s}" in output')
                break
    if expect_not_contains and ok:
        for s in (expect_not_contains if isinstance(expect_not_contains, list) else [expect_not_contains]):
            if s in output:
                ok = False
                failed += 1
                errors.append(f'FAIL [{label}]: unexpected "{s}" in output: {output!r}')
                print(f'  FAIL [{label}]: unexpected "{s}" in output')
                break
    if ok:
        passed += 1
        preview = output.strip().replace('\n', ' | ')[:80]
        print(f'  OK   [{label}]: {preview}')
    return output

def main():
    global passed, failed, errors

    print(f'Connecting to Oricutron GDB stub at {GDB_HOST}:{GDB_PORT}...')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    for attempt in range(3):
        try:
            sock.connect((GDB_HOST, GDB_PORT))
            break
        except (ConnectionRefusedError, socket.timeout) as e:
            if attempt < 2:
                print(f'  Retry {attempt+1}...')
                time.sleep(1)
                sock.close()
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
            else:
                print(f'Cannot connect: {e}')
                print('Make sure Oricutron is running with --gdb_port 6503')
                sys.exit(1)

    # Give Oricutron time to accept and pause emulation
    time.sleep(0.5)
    print('Connected.\n')

    # ---- Basic arithmetic ----
    print('=== Expression Evaluator (qOricEval) ===')
    print('--- Basic arithmetic ---')
    test_eval(sock, '5+3', 8)
    test_eval(sock, '$BB80+5', 0xBB85)
    test_eval(sock, '10-3', 7)
    test_eval(sock, '4*5', 20)
    test_eval(sock, '100/10', 10)
    test_eval(sock, '$FF&$0F', 0x0F, 'AND')
    test_eval(sock, '$AA|$55', 0xFF, 'OR')
    test_eval(sock, '$FF^$0F', 0xF0, 'XOR')

    print('--- Operator precedence ---')
    test_eval(sock, '2+3*4', 14, '2+3*4 = 14 (not 20)')
    test_eval(sock, '10-2*3', 4, '10-2*3 = 4')
    test_eval(sock, '(2+3)*4', 20, '(2+3)*4 = 20')
    test_eval(sock, '$FF&$F0|$0F', 0xFF, '$FF&$F0|$0F = $FF')

    print('--- Unary operators ---')
    test_eval(sock, '<$1234', 0x34, '<$1234 = low byte')
    test_eval(sock, '>$1234', 0x12, '>$1234 = high byte')
    test_eval(sock, '-1&$FFFF', 0xFFFF, '-1 masked to 16-bit')

    print('--- Hex, decimal, binary ---')
    test_eval(sock, '$FF', 255)
    test_eval(sock, '255', 255)
    test_eval(sock, '%11111111', 255, '%11111111 = 255')
    test_eval(sock, '%1010', 10, '%1010 = 10')

    print('--- Error cases ---')
    test_eval_fail(sock, 'nonexistent_label_xyz')
    test_eval_fail(sock, '5+')
    test_eval_fail(sock, '*5+')
    test_eval_fail(sock, '')

    # ---- Monitor commands via qOricCmd ----
    print('\n=== Monitor Commands (qOricCmd) ===')
    print('--- = command ---')
    test_cmd(sock, '= 5+3', expect_contains='$08', desc='= 5+3')
    test_cmd(sock, '= $BB80+5', expect_contains='$BB85', desc='= $BB80+5')
    test_cmd(sock, '= 2+3*4', expect_contains='$0E', desc='= 2+3*4 = $0E (14)')

    print('--- = command: no spurious symbol ---')
    test_cmd(sock, '= 5+3', expect_not_contains='volume', desc='= 5+3 no symbol noise')

    print('--- Error handling ---')
    test_cmd(sock, 'm nonexistent_xyz', expect_contains='Invalid', desc='m bad label -> error')
    test_cmd(sock, '= bad_label', expect_contains='Invalid', desc='= bad label -> error')
    test_cmd(sock, '= 5+', expect_contains='Invalid', desc='= trailing op -> error')

    print('--- d command ---')
    output = test_cmd(sock, 'd $FFFC', desc='d $FFFC (reset vector)')

    print('--- Help (?) ---')
    test_cmd(sock, '?', expect_contains='F2', desc='? shows help with F2 key')

    # ---- Summary ----
    print(f'\n{"="*50}')
    print(f'RESULTS: {passed} passed, {failed} failed')
    if errors:
        print('\nFailures:')
        for e in errors:
            print(f'  {e}')
    print(f'{"="*50}')

    sock.close()
    sys.exit(0 if failed == 0 else 1)

if __name__ == '__main__':
    main()
