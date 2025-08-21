#!/usr/bin/env python3
"""Simple OpenFIRE Dongle Console

Exactly what you asked: pick a COM port (or pass -p), then every line you type
is sent as a single ASCII string terminated by a newline ("\n").

The dongle already handles framing to the wireless devices, so we just write
plain text lines. Press Ctrl+C or type /exit to quit.

Usage:
  python simple_dongle_console.py           # list ports, prompt
  python simple_dongle_console.py -p COM7   # open COM7 directly
  python simple_dongle_console.py -p COM7 -b 9600  # specify baud if needed

Optional flags:
    --no-newline  Send exactly what you type (no automatic newline)
    --echo        Print back what was sent
    --no-rx       Disable printing incoming data (RX is ON by default)

Raw bytes:
  Use a command line starting with /raw or /hex followed by space-separated hex byte
  values (with or without 0x). Example:
      /raw 01 02 0A FF 41
  This sends: 0x01 0x02 0x0A 0xFF 0x41 (the last is 'A'). No newline is appended
  regardless of --no-newline. You can also collapse pairs: /raw 01020aff41
"""

import argparse
import sys
import time
import threading
import serial
import serial.tools.list_ports
from string import hexdigits


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
    else:
        for idx, p in enumerate(ports):
            print(f"[{idx}] {p.device:>8}  {p.description}")
    return ports


def choose_port():
    ports = list_ports()
    if not ports:
        sys.exit(1)
    while True:
        sel = input("Select port index: ")
        try:
            i = int(sel)
            if 0 <= i < len(ports):
                return ports[i].device
        except ValueError:
            pass
        print("Invalid selection.")


def rx_thread_fn(ser: serial.Serial):
    while True:
        if not ser.is_open:
            break
        try:
            data = ser.read(ser.in_waiting or 1)
            if data:
                # Print on its own line to avoid mixing with user input
                txt = data.decode(errors='replace')
                for line in txt.splitlines(True):
                    print(f"\r< {line}", end='')
                print('> ', end='', flush=True)
        except Exception:
            break
        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser(description="Simple OpenFIRE Dongle Console")
    ap.add_argument('-p', '--port', help='COM port (e.g. COM7)')
    ap.add_argument('-b', '--baud', type=int, default=9600, help='Baud rate (default 9600)')
    ap.add_argument('--no-newline', action='store_true', help='Do NOT append newline automatically')
    ap.add_argument('--echo', action='store_true', help='Echo what you send')
    ap.add_argument('--no-rx', action='store_true', help='Disable incoming data printing')
    args = ap.parse_args()

    port = args.port or choose_port()

    try:
        ser = serial.Serial(port, args.baud, timeout=0.05, write_timeout=0.5)
    except serial.SerialException as e:
        print(f"Failed to open {port}: {e}")
        return 1

    print(f"Opened {port} @ {args.baud} baud. Type /exit to quit.")
    print("Type /raw <hex bytes> to send raw bytes (e.g. /raw 01 02 0A FF).")
    if not args.no_newline:
        print("Each line you type is sent with a trailing newline.")

    rx_thread = None
    if not args.no_rx:
        rx_thread = threading.Thread(target=rx_thread_fn, args=(ser,), daemon=True)
        rx_thread.start()

    try:
        while True:
            line = input('> ')
            if line.strip().lower() in ('/exit', '/quit', 'exit', 'quit'):
                break
            # Raw hex byte sending command
            if line.startswith('/raw') or line.startswith('/hex'):
                parts = line.split(None, 1)
                if len(parts) == 1:
                    print("No hex bytes provided.")
                    continue
                spec = parts[1].strip().replace(',', ' ')
                tokens = spec.split()
                raw = bytearray()
                error = False
                for tok in tokens:
                    if tok.lower().startswith('0x'):
                        tok = tok[2:]
                    # Allow concatenated hex without spaces as one token
                    if all(c in hexdigits for c in tok):
                        if len(tok) % 2 == 1:
                            tok = '0' + tok  # pad leading zero if odd length
                        for i in range(0, len(tok), 2):
                            raw.append(int(tok[i:i+2], 16))
                    else:
                        print(f"Invalid hex token: {tok}")
                        error = True
                        break
                if error:
                    continue
                payload = bytes(raw)
            else:
                payload = line.encode('utf-8')
                if not args.no_newline:
                    payload += b'\n'
            try:
                ser.write(payload)
                ser.flush()
                if args.echo:
                    print(f"Sent {len(payload)} bytes")
            except serial.SerialTimeoutException:
                print("Write timeout.")
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        ser.close()
        print("Closed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
