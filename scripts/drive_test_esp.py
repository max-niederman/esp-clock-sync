#!/usr/bin/env python3
"""Drive a clock-sync-test ESP over serial: feed the WiFi bootstrap prompts,
then stream every line from the firmware to stdout (prefixed with the device).

Usage:
    drive_test_esp.py <port> <ssid> <password> [bssid]

If `bssid` is provided (formatted `aa:bb:cc:dd:ee:ff`) the firmware will pin
to that exact AP — important on mesh networks where otherwise different
boards land on different mesh nodes with unrelated TSF clocks.
"""

import re
import sys
import time

import serial


def main():
    if len(sys.argv) < 4:
        sys.exit(f"usage: {sys.argv[0]} <port> <ssid> <password> [bssid]")

    port_path = sys.argv[1]
    ssid = sys.argv[2]
    password = sys.argv[3]
    bssid = sys.argv[4] if len(sys.argv) >= 5 else ""

    ser = serial.Serial()
    ser.port = port_path
    ser.baudrate = 115200
    ser.dtr = False
    ser.rts = False
    ser.timeout = 0.05
    ser.open()
    # Don't toggle DTR/RTS on open — that resets the ESP.
    ser.dtr = False
    ser.rts = False

    label = port_path.split('/')[-1]
    sent = {'ssid': False, 'pass': False, 'bssid': False, 'server': False}
    buf = b''

    while True:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            # Echo what's been read, prefixing every newline.
            text = chunk.decode('utf-8', errors='replace').replace('\r', '')
            sys.stdout.write(re.sub(r'(^|\n)(?!$)', r'\1' + label + ' | ', text))
            sys.stdout.flush()

            # Detect prompts. The firmware writes "ssid: ", "pass: ", "server: ".
            tail = buf[-80:].decode('utf-8', errors='replace')
            if not sent['ssid'] and tail.endswith('ssid: '):
                ser.write((ssid + '\r\n').encode())
                ser.flush()
                sent['ssid'] = True
            elif not sent['pass'] and tail.endswith('pass: '):
                ser.write((password + '\r\n').encode())
                ser.flush()
                sent['pass'] = True
            elif not sent['bssid'] and tail.endswith('bssid: '):
                ser.write((bssid + '\r\n').encode())
                ser.flush()
                sent['bssid'] = True
            elif not sent['server'] and tail.endswith('server: '):
                ser.write(b'0.0.0.0\r\n')
                ser.flush()
                sent['server'] = True


if __name__ == '__main__':
    main()
