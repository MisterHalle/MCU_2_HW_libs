#!/usr/bin/env python3
"""
A small helper that detects when a new COM port appears on Windows and runs a command.

Requirements:
  pip install pyserial

Usage examples:
  python auto_cmd_on_com.py --cmd "echo Detectado {port}" --once
  python auto_cmd_on_com.py --cmd "esptool.py --port {port} flash_id"
  python auto_cmd_on_com.py --cmd "mode {port} BAUD=115200 PARITY=N data=8 stop=1"

Placeholders you can use inside --cmd (they will be replaced):
  {port}        -> e.g., COM3
  {port_path}   -> Windows-safe path (adds \\.\ prefix only if COM >= 10), e.g., \\.\COM12
  {friendly}    -> device description from the system
  {hwid}        -> hardware ID string

Optional filters:
  --vid 0x10C4 --pid 0xEA60   (only respond to devices with this VID:PID)

Notes:
  - By default it verifies the port can be opened before running the command (can disable with --no-verify-open).
  - On Windows, commands are executed with 'cmd /c ...' so built-in commands (like 'mode') work.
  - Use --once to exit after the first detection.
"""
import argparse
import platform
import subprocess
import sys
import time
import re

try:
    import serial
    from serial.tools import list_ports
except Exception as e:
    print("ERROR: pyserial is required. Install with: pip install pyserial", file=sys.stderr)
    raise

def win_com_path(port_name: str) -> str:
    # For COM10+ many Windows APIs require the \\.\ prefix
    m = re.fullmatch(r"COM(\d+)", port_name, flags=re.IGNORECASE)
    if m and int(m.group(1)) >= 10:
        return r"\\.\\" + port_name
    return port_name

def list_current_ports(vid=None, pid=None):
    """Return dict: {device: ListPortInfo} with optional VID/PID filtering."""
    ports = {}
    for p in list_ports.comports():
        if vid is not None and pid is not None:
            # Some drivers may not report vid/pid; skip those if filtering is enabled
            if (p.vid, p.pid) != (vid, pid):
                continue
        ports[p.device] = p
    return ports

def format_command(template: str, pinfo) -> str:
    # Simple, safe replacements without str.format() to avoid brace issues
    result = template
    result = result.replace("{port}", pinfo.device)
    result = result.replace("{port_path}", win_com_path(pinfo.device))
    result = result.replace("{friendly}", pinfo.description or "")
    result = result.replace("{hwid}", pinfo.hwid or "")
    return result

def can_open_port(port_name: str, timeout_s: float = 0.5) -> bool:
    try:
        s = serial.Serial(port=port_name, timeout=timeout_s)
        s.close()
        return True
    except Exception:
        return False

def main():
    ap = argparse.ArgumentParser(description="Watch for new COM ports and run a command")
    ap.add_argument("--cmd", required=True, help="Command to run when a new COM port appears. Use {port}, {port_path}, {friendly}, {hwid}")
    ap.add_argument("--poll", type=float, default=1.0, help="Polling interval in seconds (default: 1.0)")
    ap.add_argument("--debounce", type=float, default=1.5, help="Extra wait after detection before running the command (default: 1.5s)")
    ap.add_argument("--once", action="store_true", help="Exit after the first successful run")
    ap.add_argument("--no-verify-open", dest="verify_open", action="store_false", help="Do not try to open the port before running the command")
    ap.add_argument("--vid", type=lambda x: int(x, 0), help="USB Vendor ID filter (e.g., 0x10C4)")
    ap.add_argument("--pid", type=lambda x: int(x, 0), help="USB Product ID filter (e.g., 0xEA60)")
    ap.add_argument("--quiet", action="store_true", help="Minimal output")
    args = ap.parse_args()

    is_windows = platform.system().lower().startswith("win")
    if not is_windows:
        print("Warning: This script is intended for Windows. It may still run, but 'cmd /c' won't be used.", file=sys.stderr)

    # Snapshot current ports
    prev = list_current_ports(args.vid, args.pid)
    if not args.quiet:
        if prev:
            print(f"[init] Puertos presentes al iniciar: {', '.join(prev.keys())}")
        else:
            print("[init] No hay puertos presentes al iniciar que cumplan el filtro.")

    try:
        while True:
            time.sleep(args.poll)
            cur = list_current_ports(args.vid, args.pid)
            # New ports = present now but not in previous snapshot
            new_keys = set(cur.keys()) - set(prev.keys())
            new_ports = [cur[k] for k in new_keys]

            if new_ports:
                for pinfo in new_ports:
                    if not args.quiet:
                        print(f"[detectado] Nuevo puerto: {pinfo.device} | {pinfo.description} | {pinfo.hwid}")
                    # Debounce to allow driver stabilization
                    time.sleep(args.debounce)
                    if args.verify_open:
                        if not can_open_port(pinfo.device):
                            print(f"[aviso] No se pudo abrir {pinfo.device} aún. Reintentando en {args.poll:.1f}s...", file=sys.stderr)
                            # Try again next loop (don't mark as handled yet)
                            continue

                    cmdline = format_command(args.cmd, pinfo)
                    if not args.quiet:
                        print(f"[ejecutando] {cmdline}")

                    try:
                        if is_windows:
                            # Use cmd /c so built-ins work
                            completed = subprocess.run(["cmd", "/c", cmdline], capture_output=False, text=True, check=False)
                        else:
                            completed = subprocess.run(cmdline, shell=True, capture_output=False, text=True, check=False)
                    except Exception as e:
                        print(f"[error] Falló la ejecución del comando: {e}", file=sys.stderr)
                    else:
                        if not args.quiet:
                            print(f"[ok] Comando lanzado para {pinfo.device}.")
                        if args.once:
                            return

            prev = cur

    except KeyboardInterrupt:
        if not args.quiet:
            print("\n[fin] Interrumpido por el usuario.")

if __name__ == "__main__":
    main()
