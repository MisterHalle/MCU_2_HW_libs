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

HYBRID_RE = re.compile(
    r"\[HYBRID\] "
    r"IBI=(?P<ibi>\d+) Q=(?P<ibi_q>\d+) \| "
    r"CORR=(?P<corr>\d+) Q=(?P<corr_q>\d+) R=(?P<corr_r>[\d.]+) \| "
    r"SPEC=(?P<spec>\d+) RAW=(?P<spec_raw>\d+) Q=(?P<spec_q>\d+) "
    r"DOM=(?P<dom>[\d.]+) H2=(?P<h2>[\d.]+) \| "
    r"FUSED=(?P<fused>\d+) Q=(?P<fuse_q>\d+) METHODS=(?P<methods>\d+) "
    r"VALID=(?P<valid>YES|NO) HARMONIC=(?P<harmonic>YES|NO) "
    r"SAMPLES=(?P<samples>\d+)"
)

FIELDS = [
    "pc_datetime", "elapsed_ms", "record_type",
    "ibi_bpm", "ibi_quality",
    "corr_bpm", "corr_quality", "corr_strength",
    "spectral_bpm", "spectral_raw_bpm", "spectral_quality",
    "spectral_dominance", "second_harmonic_ratio",
    "fused_bpm", "fusion_confidence", "methods_agree",
    "fusion_valid", "harmonic_suspect", "hybrid_samples",
    "serial_line",
]

def choose_port():
    ports = list(list_ports.comports())
    if not ports:
        return None
    if len(ports) == 1:
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

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--output")
    args = ap.parse_args()

    port = args.port or choose_port()
    if not port:
        return

    if args.output:
        output = Path(args.output)
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = Path(f"APDS9008_hybrid_{stamp}.csv")

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
                line = ser.readline().decode(
                    "utf-8", errors="replace"
                ).strip()

                if not line:
                    continue

                elapsed = int((time.perf_counter() - start) * 1000)
                m = HYBRID_RE.fullmatch(line)

                row = {
                    "pc_datetime": datetime.now().isoformat(
                        timespec="milliseconds"
                    ),
                    "elapsed_ms": elapsed,
                    "serial_line": line,
                }

                if m:
                    g = m.groupdict()
                    row.update({
                        "record_type": "HYBRID",
                        "ibi_bpm": g["ibi"],
                        "ibi_quality": g["ibi_q"],
                        "corr_bpm": g["corr"],
                        "corr_quality": g["corr_q"],
                        "corr_strength": g["corr_r"],
                        "spectral_bpm": g["spec"],
                        "spectral_raw_bpm": g["spec_raw"],
                        "spectral_quality": g["spec_q"],
                        "spectral_dominance": g["dom"],
                        "second_harmonic_ratio": g["h2"],
                        "fused_bpm": g["fused"],
                        "fusion_confidence": g["fuse_q"],
                        "methods_agree": g["methods"],
                        "fusion_valid": 1 if g["valid"] == "YES" else 0,
                        "harmonic_suspect": 1 if g["harmonic"] == "YES" else 0,
                        "hybrid_samples": g["samples"],
                    })
                    hybrid_count += 1
                else:
                    row["record_type"] = "RAW_LINE"
                    raw_count += 1

                w.writerow(row)
                f.flush()

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    print(f"HYBRID: {hybrid_count}")
    print(f"RAW_LINE: {raw_count}")
    print(output.resolve())

if __name__ == "__main__":
    main()
