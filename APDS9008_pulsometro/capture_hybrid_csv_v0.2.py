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
    print("Falta pyserial: py -m pip install pyserial")
    raise SystemExit(1)

FIELDS = [
    "pc_datetime", "elapsed_ms", "record_type",
    "ibi_bpm", "ibi_quality", "ibi_relation",
    "corr_bpm", "corr_quality", "corr_strength", "corr_relation",
    "spectral_bpm", "spectral_raw_bpm", "spectral_quality",
    "spectral_dominance", "second_harmonic_ratio", "spectral_relation",
    "fused_bpm", "fusion_confidence", "methods_agree",
    "fusion_valid", "harmonic_suspect", "harmonic_resolved", "fusion_score",
    "hybrid_samples", "analysis_cpu_us", "analysis_max_cpu_us",
    "slice_us", "slice_max_us", "cycle_elapsed_ms", "sample_gap_max_ms",
    "serial_line",
]

def choose_port():
    ports = list(list_ports.comports())
    if not ports:
        return None
    if len(ports) == 1:
        print(f"Usando {ports[0].device} - {ports[0].description}")
        return ports[0].device
    for i, p in enumerate(ports, 1):
        print(f"{i}. {p.device} - {p.description}")
    while True:
        s = input("Puerto: ").strip()
        if not s:
            return None
        try:
            i = int(s)
            if 1 <= i <= len(ports):
                return ports[i - 1].device
        except ValueError:
            pass

def grab(pattern, line, cast=str, default=""):
    m = re.search(pattern, line)
    if not m:
        return default
    try:
        return cast(m.group(1))
    except Exception:
        return default

def parse_hybrid(line):
    if not line.startswith("[HYBRID]"):
        return None

    # Separamos bloques para que Q/REL repetidos no se confundan.
    parts = line[len("[HYBRID]"):].strip().split(" | ")
    if len(parts) < 4:
        return None

    ibi, corr, spec, tail = parts[0], parts[1], parts[2], " | ".join(parts[3:])

    row = {
        "record_type": "HYBRID",
        "ibi_bpm": grab(r"IBI=(\d+)", ibi, int),
        "ibi_quality": grab(r"Q=(\d+)", ibi, int),
        "ibi_relation": grab(r"REL=([^ ]+)", ibi),

        "corr_bpm": grab(r"CORR=(\d+)", corr, int),
        "corr_quality": grab(r"Q=(\d+)", corr, int),
        "corr_strength": grab(r"R=([0-9.]+)", corr, float),
        "corr_relation": grab(r"REL=([^ ]+)", corr),

        "spectral_bpm": grab(r"SPEC=(\d+)", spec, int),
        "spectral_raw_bpm": grab(r"RAW=(\d+)", spec, int),
        "spectral_quality": grab(r"Q=(\d+)", spec, int),
        "spectral_dominance": grab(r"DOM=([0-9.]+)", spec, float),
        "second_harmonic_ratio": grab(r"H2=([0-9.]+)", spec, float),
        "spectral_relation": grab(r"REL=([^ ]+)", spec),

        "fused_bpm": grab(r"FUSED=(\d+)", tail, int),
        "fusion_confidence": grab(r"Q=(\d+)", tail, int),
        "methods_agree": grab(r"METHODS=(\d+)", tail, int),
        "fusion_valid": 1 if grab(r"VALID=(YES|NO)", tail) == "YES" else 0,
        "harmonic_suspect": 1 if grab(r"HARMONIC=(YES|NO)", tail) == "YES" else 0,
        "harmonic_resolved": 1 if grab(r"RESOLVED=(YES|NO)", tail) == "YES" else 0,
        "fusion_score": grab(r"SCORE=([0-9.]+)", tail, float),
        "hybrid_samples": grab(r"SAMPLES=(\d+)", tail, int),
        "analysis_cpu_us": grab(r"CPU=(\d+)us", tail, int),
        "analysis_max_cpu_us": grab(r"MAXCPU=(\d+)us", tail, int),
        "slice_us": grab(r"SLICE=(\d+)us", tail, int),
        "slice_max_us": grab(r"MAXSLICE=(\d+)us", tail, int),
        "cycle_elapsed_ms": grab(r"CYCLE=(\d+)ms", tail, int),
        "sample_gap_max_ms": grab(r"GAPMAX=(\d+)ms", tail, int),
    }
    return row

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--output")
    args = ap.parse_args()

    port = args.port or choose_port()
    if not port:
        return

    output = Path(args.output) if args.output else Path(
        f"APDS9008_hybrid_v053_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

    ser = serial.Serial(port, args.baud, timeout=1)
    time.sleep(1.5)
    ser.reset_input_buffer()

    start = time.perf_counter()
    hybrid_count = 0
    raw_count = 0

    try:
        with output.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=FIELDS)
            w.writeheader()
            f.flush()

            while True:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                elapsed = int((time.perf_counter() - start) * 1000)
                parsed = parse_hybrid(line)

                row = {
                    "pc_datetime": datetime.now().isoformat(timespec="milliseconds"),
                    "elapsed_ms": elapsed,
                    "serial_line": line,
                }

                if parsed is not None:
                    row.update(parsed)
                    hybrid_count += 1
                else:
                    row["record_type"] = "RAW_LINE"
                    raw_count += 1

                w.writerow(row)
                f.flush()

    except KeyboardInterrupt:
        print("\nCaptura detenida.")
    finally:
        ser.close()

    print(f"HYBRID: {hybrid_count}")
    print(f"RAW_LINE: {raw_count}")
    print(output.resolve())

if __name__ == "__main__":
    main()
