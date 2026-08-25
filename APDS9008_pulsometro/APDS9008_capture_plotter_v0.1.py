import argparse
import csv
import re
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("")
    print("ERROR: falta pyserial.")
    print("Instálalo con:")
    print("  py -m pip install pyserial")
    print("")
    raise SystemExit(1)

BAUD = 115200
PLOT_CENTER = 2048.0

LINE_RE = re.compile(
    r"RAW:(?P<raw>-?\d+(?:\.\d+)?)"
    r",DC:(?P<dc>-?\d+(?:\.\d+)?)"
    r",AC:(?P<ac>-?\d+(?:\.\d+)?)"
    r",ENV:(?P<env>-?\d+(?:\.\d+)?)"
    r",THR_POS:(?P<thr_pos>-?\d+(?:\.\d+)?)"
    r",THR_NEG:(?P<thr_neg>-?\d+(?:\.\d+)?)"
    r",REARM:(?P<rearm>-?\d+(?:\.\d+)?)"
)

FIELDS = [
    "pc_datetime",
    "elapsed_ms",
    "sample_number",
    "record_type",
    "raw",
    "dc",
    "ac_plot",
    "env_plot",
    "thr_pos_plot",
    "thr_neg_plot",
    "rearm_plot",
    "ac_real",
    "env_real",
    "threshold_real",
    "threshold_negative_real",
    "rearm_real",
    "serial_line",
]

def choose_port():
    ports = list(list_ports.comports())

    if not ports:
        print("No se encontraron puertos COM.")
        return None

    if len(ports) == 1:
        print(f"Usando {ports[0].device} - {ports[0].description}")
        return ports[0].device

    print("\nPuertos disponibles:\n")
    for i, p in enumerate(ports, 1):
        print(f"  {i}. {p.device} - {p.description}")

    while True:
        value = input("\nSelecciona puerto (ENTER cancela): ").strip()

        if not value:
            return None

        try:
            idx = int(value)
            if 1 <= idx <= len(ports):
                return ports[idx - 1].device
        except ValueError:
            pass

        print("Selección inválida.")

def parse_line(line):
    m = LINE_RE.fullmatch(line.strip())

    if not m:
        return None

    d = {k: float(v) for k, v in m.groupdict().items()}

    return {
        "record_type": "PLOTTER",
        "raw": d["raw"],
        "dc": d["dc"],
        "ac_plot": d["ac"],
        "env_plot": d["env"],
        "thr_pos_plot": d["thr_pos"],
        "thr_neg_plot": d["thr_neg"],
        "rearm_plot": d["rearm"],
        "ac_real": d["ac"] - PLOT_CENTER,
        "env_real": d["env"] - PLOT_CENTER,
        "threshold_real": d["thr_pos"] - PLOT_CENTER,
        "threshold_negative_real": d["thr_neg"] - PLOT_CENTER,
        "rearm_real": d["rearm"] - PLOT_CENTER,
    }

def empty_numeric_row():
    return {
        "raw": "",
        "dc": "",
        "ac_plot": "",
        "env_plot": "",
        "thr_pos_plot": "",
        "thr_neg_plot": "",
        "rearm_plot": "",
        "ac_real": "",
        "env_real": "",
        "threshold_real": "",
        "threshold_negative_real": "",
        "rearm_real": "",
    }

def main():
    parser = argparse.ArgumentParser(
        description="Captura APDS-9008 Serial Plotter a CSV"
    )
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--output")
    args = parser.parse_args()

    port = args.port or choose_port()

    if not port:
        print("Captura cancelada.")
        return

    if args.output:
        output = Path(args.output)
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = Path(f"APDS9008_capture_{stamp}.csv")

    print("\n==============================================")
    print(" APDS-9008 CSV CAPTURE v0.2 FAILSAFE")
    print("==============================================")
    print(f"Puerto : {port}")
    print(f"Baud   : {args.baud}")
    print(f"CSV    : {output.resolve()}")
    print("\nFirmware:")
    print("  SERIAL_PLOTTER_MODE = true")
    print("\nCierra Serial Monitor/Plotter antes de iniciar.")
    print("Ctrl+C para detener.")
    print("==============================================\n")

    try:
        ser = serial.Serial(port, args.baud, timeout=1)
    except serial.SerialException as exc:
        print(f"No se pudo abrir {port}: {exc}")
        return

    time.sleep(1.5)
    ser.reset_input_buffer()

    start = time.perf_counter()
    sample_number = 0
    parsed_count = 0
    raw_line_count = 0

    try:
        with output.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=FIELDS)
            writer.writeheader()
            f.flush()

            while True:
                try:
                    data = ser.readline()
                except serial.SerialException as exc:
                    print(f"\nError serial: {exc}")
                    break

                if not data:
                    continue

                line = data.decode("utf-8", errors="replace").strip()

                if not line:
                    continue

                sample_number += 1
                elapsed_ms = int((time.perf_counter() - start) * 1000)
                parsed = parse_line(line)

                if parsed is None:
                    raw_line_count += 1
                    row = {
                        "pc_datetime": datetime.now().isoformat(
                            timespec="milliseconds"
                        ),
                        "elapsed_ms": elapsed_ms,
                        "sample_number": sample_number,
                        "record_type": "RAW_LINE",
                        **empty_numeric_row(),
                        "serial_line": line,
                    }
                else:
                    parsed_count += 1
                    row = {
                        "pc_datetime": datetime.now().isoformat(
                            timespec="milliseconds"
                        ),
                        "elapsed_ms": elapsed_ms,
                        "sample_number": sample_number,
                        **parsed,
                        "serial_line": line,
                    }

                writer.writerow(row)
                f.flush()

                if parsed is not None and parsed_count % 100 == 0:
                    print(
                        f"válidas={parsed_count:6d}  "
                        f"t={elapsed_ms/1000:6.1f}s  "
                        f"RAW={parsed['raw']:6.0f}  "
                        f"AC={parsed['ac_real']:+7.1f}  "
                        f"ENV={parsed['env_real']:6.1f}  "
                        f"THR={parsed['threshold_real']:6.1f}",
                        end="\r",
                        flush=True,
                    )

    except KeyboardInterrupt:
        print("\n\nCaptura detenida.")

    finally:
        ser.close()

    duration = time.perf_counter() - start

    print("\n==============================================")
    print(" RESUMEN")
    print("==============================================")
    print(f"Duración              : {duration:.2f} s")
    print(f"Muestras reconocidas  : {parsed_count}")
    print(f"Líneas RAW_LINE       : {raw_line_count}")

    if duration > 0:
        print(f"Frecuencia reconocida : {parsed_count/duration:.1f} Hz")

    print(f"Archivo               : {output.resolve()}")

    if parsed_count == 0:
        print("\nADVERTENCIA:")
        print("No se reconoció ninguna línea PLOTTER.")
        print("Las líneas recibidas igualmente quedaron guardadas")
        print("como RAW_LINE para poder revisar el formato.")

    print("==============================================\n")

if __name__ == "__main__":
    main()