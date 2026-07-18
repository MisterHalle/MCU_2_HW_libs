"""
MAX30102 Serial Logger Rebased OpticalGate v1.1.3

Guarda únicamente filas CSV completas de 41 columnas y permite marcar:
    Q = comienza QUIETUD
    M = comienza MOVIMIENTO

Requisito:
    python -m pip install pyserial
"""

from __future__ import annotations

import os
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial import SerialException
    from serial.tools import list_ports
except ImportError:
    print("[ERROR] Falta instalar pyserial.")
    print("Ejecuta: python -m pip install pyserial")
    sys.exit(1)

BAUDRATE = 115200
SERIAL_TIMEOUT_S = 0.25
OUTPUT_FOLDER = "capturas_max30102_opticalgate"
EXPECTED_COLUMNS = 41

COLUMN_HEADER = (
    "ts_ms,red,ir,red_f,ir_f,thr,quality,pi,corr,gain,gain_dev,"
    "optical_ok,state,quiet_ok,motion,candidate,beat,ibi_ms,polarity,"
    "temporal_conf,morph_conf,min_temporal,min_morph,reject,recovered_n,"
    "hr,hr_valid,hr_provisional,hr_held,hr_provisional_bpm,ref_ibi_ms,"
    "red_range_pct,ir_range_pct,R,SpO2,usable,stableN,adc,redPA,irPA,adj"
)

STATE_NAMES = {
    "0": "MOVING",
    "1": "SETTLING",
    "2": "ACQUIRING",
    "3": "TRACKING",
}


def choose_port() -> str:
    ports = list(list_ports.comports())
    print("\nPuertos serie detectados:")
    for index, port in enumerate(ports, start=1):
        print(f"  {index}) {port.device} | {port.description or 'Sin descripción'}")

    while True:
        selection = input("\nSelecciona número o escribe COMx: ").strip()
        if not selection:
            continue
        if selection.isdigit() and ports:
            index = int(selection)
            if 1 <= index <= len(ports):
                return ports[index - 1].device
        return selection


def create_output_paths() -> tuple[Path, Path]:
    output_dir = Path(__file__).resolve().parent / OUTPUT_FOLDER
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = f"MAX30102_OpticalGate_{stamp}"
    return output_dir / f"{base}.txt", output_dir / f"{base}_events.txt"


def read_hotkey() -> str | None:
    if os.name != "nt":
        return None
    import msvcrt

    if not msvcrt.kbhit():
        return None
    key = msvcrt.getwch().lower()
    return key if key in {"q", "m"} else None


def main() -> None:
    print("====================================================")
    print(" MAX30102 Logger Rebased OpticalGate v1.1.3")
    print("====================================================")

    port_name = choose_port()
    data_path, event_path = create_output_paths()
    start_time = time.monotonic()
    valid_rows = 0
    discarded_rows = 0
    last_sensor_ts = 0
    last_state = ""

    try:
        with serial.Serial(port_name, BAUDRATE, timeout=SERIAL_TIMEOUT_S) as port:
            time.sleep(2.0)
            port.reset_input_buffer()

            print(f"\n[OK] Puerto: {port_name} @ {BAUDRATE}")
            print(f"[OK] Datos:   {data_path}")
            print(f"[OK] Eventos: {event_path}")
            print("[MARCAS] Q = QUIETUD | M = MOVIMIENTO | Ctrl+C = finalizar\n")

            with (
                data_path.open("w", encoding="utf-8", newline="\n") as data_file,
                event_path.open("w", encoding="utf-8", newline="\n") as event_file,
            ):
                data_file.write(COLUMN_HEADER + "\n")
                event_file.write("host_elapsed_s,sensor_ts_ms,event\n")
                data_file.flush()
                event_file.flush()

                while True:
                    key = read_hotkey()
                    if key:
                        event = "QUIET_START" if key == "q" else "MOTION_START"
                        elapsed = time.monotonic() - start_time
                        event_file.write(f"{elapsed:.3f},{last_sensor_ts},{event}\n")
                        event_file.flush()
                        print(f"[EVENTO] {event} | sensor_ts={last_sensor_ts}")

                    raw = port.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue

                    fields = line.split(",")
                    if not fields[0].strip().isdigit():
                        print(f"[INFO] {line}")
                        continue
                    if len(fields) != EXPECTED_COLUMNS:
                        discarded_rows += 1
                        print(
                            f"[DESCARTADA] {len(fields)} columnas; "
                            f"se esperaban {EXPECTED_COLUMNS}."
                        )
                        continue

                    try:
                        last_sensor_ts = int(fields[0])
                        state = fields[12]
                        beat = fields[16] == "1"
                        hr = float(fields[25])
                        valid = fields[26] == "1"
                        provisional = fields[27] == "1"
                        temporal = float(fields[19])
                        morphology = float(fields[20])
                    except (ValueError, IndexError):
                        discarded_rows += 1
                        continue

                    data_file.write(line + "\n")
                    valid_rows += 1

                    if state != last_state:
                        print(
                            f"[ESTADO] {STATE_NAMES.get(state, state)} | "
                            f"sensor_ts={last_sensor_ts}"
                        )
                        last_state = state

                    if beat:
                        label = "VALIDO" if valid else "ACEPTADO"
                        print(
                            f"[PULSO {label}] HR={hr:.1f} | "
                            f"T={temporal:.1f} M={morphology:.1f}"
                        )
                    elif provisional:
                        print(
                            f"[PROVISIONAL] HR={hr:.1f} | "
                            f"T={temporal:.1f} M={morphology:.1f}"
                        )

                    if valid_rows % 100 == 0:
                        data_file.flush()
                        elapsed = time.monotonic() - start_time
                        print(
                            f"[CAPTURA] {valid_rows} filas | {elapsed:.1f} s | "
                            f"descartadas={discarded_rows}"
                        )

    except KeyboardInterrupt:
        elapsed = time.monotonic() - start_time
        print("\n[FIN] Captura detenida.")
        print(f"[FIN] Filas: {valid_rows} | descartadas: {discarded_rows}")
        print(f"[FIN] Duración: {elapsed:.1f} s")
        print(f"[FIN] Datos:   {data_path}")
        print(f"[FIN] Eventos: {event_path}")
    except SerialException as exc:
        print(f"\n[ERROR SERIAL] {exc}")
        print("Cierra el Monitor Serial de Arduino y revisa el puerto COM.")
        sys.exit(1)
    except OSError as exc:
        print(f"\n[ERROR DE ARCHIVO] {exc}")
        sys.exit(1)


if __name__ == "__main__":
    main()