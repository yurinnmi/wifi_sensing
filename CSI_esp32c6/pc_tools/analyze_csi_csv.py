# 保存したCSVを読み込み、以下を表示します。
#  RSSI時系列
#  amp_avg 時系列
#  amp_max 時系列
#  DeltaScore 時系列
#  サブキャリア相当の振幅ヒートマップ
#  基準との差分ヒートマップ

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)

    required = ["ms", "count", "rssi", "len", "pair_count", "amp_avg", "amp_max"]
    missing = [col for col in required if col not in df.columns]

    if missing:
        raise ValueError(f"Missing required columns: {missing}")

    for col in required:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df.dropna(subset=["ms", "count"])
    df = df.reset_index(drop=True)

    if df.empty:
        raise ValueError("CSV has no valid data rows.")

    return df


def get_amp_columns(df: pd.DataFrame) -> list[str]:
    amp_cols = [col for col in df.columns if col.startswith("amp")]

    # amp_avg / amp_max は除外
    amp_cols = [col for col in amp_cols if col not in ("amp_avg", "amp_max")]

    # amp0, amp1, ... の数値順に並べる
    def amp_index(name: str) -> int:
        return int(name.replace("amp", ""))

    amp_cols.sort(key=amp_index)

    if not amp_cols:
        raise ValueError("No amp0, amp1, ... columns found.")

    return amp_cols


def build_time_sec(df: pd.DataFrame) -> np.ndarray:
    ms = df["ms"].to_numpy(dtype=float)
    return (ms - ms[0]) / 1000.0


def build_amp_matrix(df: pd.DataFrame, amp_cols: list[str]) -> np.ndarray:
    amp_df = df[amp_cols].apply(pd.to_numeric, errors="coerce")

    # 全部NaNの列は落とす
    amp_df = amp_df.dropna(axis=1, how="all")

    if amp_df.empty:
        raise ValueError("All amp columns are empty.")

    return amp_df.to_numpy(dtype=float)


def calculate_delta_score(
    amp_matrix: np.ndarray,
    time_sec: np.ndarray,
    baseline_sec: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    baseline_mask = time_sec <= baseline_sec

    if not np.any(baseline_mask):
        raise ValueError("No rows in baseline period. Increase --baseline-sec.")

    baseline = np.nanmean(amp_matrix[baseline_mask, :], axis=0)

    # 基準がNaNの列は後段でNaNにする
    denom = baseline + 1.0

    diff = np.abs(amp_matrix - baseline)

    # サブキャリアごとの正規化差分[%っぽい値]
    delta_matrix = 100.0 * diff / denom

    # 各時刻の平均差分
    delta_score = np.nanmean(delta_matrix, axis=1)

    return delta_score, delta_matrix, baseline


def plot_series(time_sec: np.ndarray, y: np.ndarray, title: str, ylabel: str) -> None:
    plt.figure()
    plt.plot(time_sec, y)
    plt.xlabel("Time [s]")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)


def plot_heatmap(
    time_sec: np.ndarray,
    matrix: np.ndarray,
    title: str,
    colorbar_label: str,
) -> None:
    plt.figure()

    if len(time_sec) >= 2:
        extent = [time_sec[0], time_sec[-1], 0, matrix.shape[1] - 1]
    else:
        extent = [0, 1, 0, matrix.shape[1] - 1]

    plt.imshow(
        matrix.T,
        aspect="auto",
        origin="lower",
        extent=extent,
        interpolation="nearest",
    )
    plt.xlabel("Time [s]")
    plt.ylabel("I/Q pair index")
    plt.title(title)
    plt.colorbar(label=colorbar_label)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze ESP32-C6 CSI CSV and visualize RSSI, amplitude, DeltaScore, and heatmaps."
    )
    parser.add_argument(
        "csv",
        help="Input CSV file recorded by csi_serial_logger.py.",
    )
    parser.add_argument(
        "--baseline-sec",
        type=float,
        default=5.0,
        help="Initial seconds used as baseline. Default: 5.0",
    )
    parser.add_argument(
        "--save-plots",
        action="store_true",
        help="Save plots as PNG files next to the CSV.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show plots interactively.",
    )

    args = parser.parse_args()

    csv_path = Path(args.csv)
    df = load_csv(csv_path)

    main_pair_count = df["pair_count"].mode().iloc[0]
    df = df[df["pair_count"] == main_pair_count].reset_index(drop=True)
    print(f"Using pair_count={main_pair_count} rows only.")

    time_sec = build_time_sec(df)

    amp_cols = get_amp_columns(df)
    amp_matrix = build_amp_matrix(df, amp_cols)

    delta_score, delta_matrix, baseline = calculate_delta_score(
        amp_matrix,
        time_sec,
        args.baseline_sec,
    )

    print("CSV:", csv_path)
    print("Rows:", len(df))
    print("Amp columns:", amp_matrix.shape[1])
    print("Time range [s]:", time_sec[0], "to", time_sec[-1])
    print("Baseline seconds:", args.baseline_sec)
    print("DeltaScore mean:", float(np.nanmean(delta_score)))
    print("DeltaScore max :", float(np.nanmax(delta_score)))

    # 1. RSSI
    plot_series(
        time_sec,
        df["rssi"].to_numpy(dtype=float),
        "RSSI over time",
        "RSSI [dBm]",
    )

    # 2. amp_avg
    plot_series(
        time_sec,
        df["amp_avg"].to_numpy(dtype=float),
        "CSI average amplitude over time",
        "amp_avg",
    )

    # 3. amp_max
    plot_series(
        time_sec,
        df["amp_max"].to_numpy(dtype=float),
        "CSI max amplitude over time",
        "amp_max",
    )

    # 4. DeltaScore
    plot_series(
        time_sec,
        delta_score,
        "DeltaScore over time",
        "DeltaScore",
    )

    # 5. 振幅ヒートマップ
    plot_heatmap(
        time_sec,
        amp_matrix,
        "CSI amplitude heatmap",
        "Amplitude",
    )

    # 6. 基準との差分ヒートマップ
    plot_heatmap(
        time_sec,
        delta_matrix,
        "CSI delta heatmap from baseline",
        "Delta [%]",
    )

    if args.save_plots:
        out_dir = csv_path.with_suffix("")
        out_dir.mkdir(exist_ok=True)

        figure_names = [
            "01_rssi.png",
            "02_amp_avg.png",
            "03_amp_max.png",
            "04_delta_score.png",
            "05_amp_heatmap.png",
            "06_delta_heatmap.png",
        ]

        for fig_num, name in enumerate(figure_names, start=1):
            plt.figure(fig_num)
            plt.tight_layout()
            out_path = out_dir / name
            plt.savefig(out_path, dpi=150)
            print("Saved:", out_path)

    if args.show or not args.save_plots:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())