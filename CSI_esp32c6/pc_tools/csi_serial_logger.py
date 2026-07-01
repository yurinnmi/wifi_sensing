import argparse
import datetime as dt
import sys
from pathlib import Path

import serial


def make_default_filename() -> str:
    now = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"csi_log_{now}.csv"


def is_data_line(line: str) -> bool:
    """
    CSVデータ行かどうかをざっくり判定する。
    期待形式:
      ms,count,rssi,len,pair_count,amp_avg,amp_max,amp0,...
    """
    if not line:
        return False

    if line.startswith("#"):
        return False

    if line.startswith("ms,count,rssi"):
        return False

    parts = line.split(",")

    if len(parts) < 7:
        return False

    try:
        int(parts[0])    # ms
        int(parts[1])    # count
        int(parts[2])    # rssi
        int(parts[3])    # len
        int(parts[4])    # pair_count
        float(parts[5])  # amp_avg
        float(parts[6])  # amp_max
    except ValueError:
        return False

    return True


def make_header_from_data_line(line: str) -> str:
    """
    データ行の列数からCSVヘッダを自動生成する。
    """
    parts = line.split(",")

    fixed_cols = [
        "ms",
        "count",
        "rssi",
        "len",
        "pair_count",
        "amp_avg",
        "amp_max",
    ]

    amp_count = len(parts) - len(fixed_cols)

    amp_cols = [f"amp{i}" for i in range(amp_count)]

    return ",".join(fixed_cols + amp_cols)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--out", default=make_default_filename())
    args = parser.parse_args()

    out_path = Path(args.out)

    print(f"Opening serial port: {args.port} @ {args.baud}")
    print(f"Saving CSV to: {out_path}")
    print("Press Ctrl+C to stop.")

    got_header = False
    csv_lines = 0
    rx_lines = 0

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as ser:
            with out_path.open("w", encoding="utf-8", newline="") as f:
                while True:
                    raw = ser.readline()

                    if not raw:
                        continue

                    line = raw.decode("utf-8", errors="replace").strip()

                    if not line:
                        continue

                    rx_lines += 1
                    print(line)

                    # ESP32側のコメント行は保存しない
                    if line.startswith("#"):
                        continue

                    # 通常のCSVヘッダを受信した場合
                    if line.startswith("ms,count,rssi"):
                        if not got_header:
                            f.write(line + "\n")
                            f.flush()
                            got_header = True
                            csv_lines += 1
                            print("CSV header saved.")
                        continue

                    # データ行を受信した場合
                    if is_data_line(line):
                        # ヘッダを見逃していた場合、データ行から自動生成する
                        if not got_header:
                            header = make_header_from_data_line(line)
                            f.write(header + "\n")
                            f.flush()
                            got_header = True
                            csv_lines += 1
                            print("CSV header auto-generated.")

                        f.write(line + "\n")
                        f.flush()
                        csv_lines += 1
                        continue

                    # その他の行は保存しない
                    print("Ignored non-CSV line.")

    except KeyboardInterrupt:
        print()
        print("Stopped.")
        print(f"RX lines saved/check : {rx_lines}")
        print(f"CSV lines saved      : {csv_lines}")
        print(f"Saved file           : {out_path}")

        if csv_lines <= 1:
            print("CSV has no data rows.")
            print("ESP32-C6 may not be outputting CSI data rows.")

        return 0

    except serial.SerialException as e:
        print(f"Serial error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())