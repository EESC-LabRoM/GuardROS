#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
analyze_guardros_logs.py

Statistical analysis and plotting script for GuardROS experiment logs.

This script reads one GuardROS logging session and generates:
1. CSV statistical summaries.
2. A human-readable TXT report.
3. PNG plots for timing, jitter, telemetry, battery, temperature, and ping.

It is independent from ROS 2 and should be executed from the workspace root.

Example:
--------
cd /home/denis/DENIS/RollerBot23/guardros_ws

python3 experiments/analysis/analyze_guardros_logs.py \
    experiments/logs/2026-05-11_22-56-26_manual_run
"""

import json
import sys
from pathlib import Path

import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


EXPECTED_PERIODS_SEC = {
    "cmd_vel": 0.05,
    "telemetry_json": 0.10,
    "video": 0.05,
    "audio_rx": 0.17,
    "audio_tx": 0.0213,
}


def read_csv_safe(path: Path) -> pd.DataFrame:
    if not path.exists():
        print(f"[WARNING] Missing file: {path.name}")
        return pd.DataFrame()
    try:
        return pd.read_csv(path)
    except Exception as exc:
        print(f"[WARNING] Could not read {path.name}: {exc}")
        return pd.DataFrame()


def read_jsonl_safe(path: Path) -> pd.DataFrame:
    if not path.exists():
        print(f"[WARNING] Missing file: {path.name}")
        return pd.DataFrame()

    rows = []
    try:
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rows.append(json.loads(line))
                except Exception:
                    continue
        return pd.DataFrame(rows)
    except Exception as exc:
        print(f"[WARNING] Could not read {path.name}: {exc}")
        return pd.DataFrame()


def stability_label(p99_dt, max_dt, expected_period, long_gap_count) -> str:
    if p99_dt is None or max_dt is None or expected_period is None:
        return "not_available"
    if long_gap_count == 0 and p99_dt <= 2.0 * expected_period:
        return "excellent"
    if p99_dt <= 3.0 * expected_period:
        return "good"
    if p99_dt <= 5.0 * expected_period:
        return "acceptable"
    return "needs_investigation"


def basic_rate_stats(df: pd.DataFrame, name: str) -> dict:
    expected_period = EXPECTED_PERIODS_SEC.get(name)

    result = {
        "stream": name,
        "message_count": 0,
        "duration_sec": None,
        "average_rate_hz": None,
        "expected_period_sec": expected_period,
        "expected_rate_hz": None if expected_period is None else 1.0 / expected_period,
        "mean_dt_sec": None,
        "median_dt_sec": None,
        "std_dt_sec": None,
        "min_dt_sec": None,
        "max_dt_sec": None,
        "p95_dt_sec": None,
        "p99_dt_sec": None,
        "gaps_above_2x_expected": None,
        "gaps_above_0_2_sec": None,
        "stability_label": "not_available",
    }

    if df.empty:
        return result

    result["message_count"] = len(df)

    if "elapsed_sec" in df.columns:
        elapsed = pd.to_numeric(df["elapsed_sec"], errors="coerce").dropna()
        if len(elapsed) > 0:
            duration = float(elapsed.max() - elapsed.min())
            result["duration_sec"] = duration
            if duration > 0:
                result["average_rate_hz"] = float(len(df) / duration)

    if "dt_since_previous_sec" in df.columns:
        dt = pd.to_numeric(df["dt_since_previous_sec"], errors="coerce").dropna()
        if len(dt) > 0:
            result["mean_dt_sec"] = float(dt.mean())
            result["median_dt_sec"] = float(dt.median())
            result["std_dt_sec"] = float(dt.std())
            result["min_dt_sec"] = float(dt.min())
            result["max_dt_sec"] = float(dt.max())
            result["p95_dt_sec"] = float(dt.quantile(0.95))
            result["p99_dt_sec"] = float(dt.quantile(0.99))
            result["gaps_above_0_2_sec"] = int((dt > 0.2).sum())

            if expected_period is not None:
                result["gaps_above_2x_expected"] = int((dt > 2.0 * expected_period).sum())

            result["stability_label"] = stability_label(
                result["p99_dt_sec"],
                result["max_dt_sec"],
                expected_period,
                result["gaps_above_0_2_sec"],
            )

    return result


def summarize_variables(typed_df: pd.DataFrame) -> list[dict]:
    variables = [
        "voltage",
        "current",
        "power",
        "percentage",
        "pcb_temp",
        "cpu_temp",
        "robot_server_ping",
        "client_server_ping",
    ]

    rows = []
    if typed_df.empty:
        return rows

    for var in variables:
        if var not in typed_df.columns:
            continue

        values = pd.to_numeric(typed_df[var], errors="coerce").dropna()
        if len(values) == 0:
            continue

        rows.append(
            {
                "variable": var,
                "count": int(len(values)),
                "mean": float(values.mean()),
                "median": float(values.median()),
                "std": float(values.std()),
                "min": float(values.min()),
                "max": float(values.max()),
                "p95": float(values.quantile(0.95)),
                "p99": float(values.quantile(0.99)),
            }
        )

    return rows


def summarize_driver_diagnostics(driver_df: pd.DataFrame) -> dict:
    summary = {}
    if driver_df.empty:
        return summary

    last = driver_df.iloc[-1]

    fields = [
        "udp_command_packets_sent",
        "udp_command_send_errors",
        "udp_audio_packets_sent",
        "udp_audio_send_errors",
        "udp_packets_rx_total",
        "udp_socket_timeouts",
        "udp_recv_errors",
        "udp_unknown_packets",
        "telemetry_packets_rx",
        "telemetry_packets_published",
        "telemetry_packets_wrong_id",
        "telemetry_decode_attempts_ok",
        "telemetry_decode_attempts_failed",
        "video_packets_ok",
        "video_packets_non_jpeg",
        "audio_packets_ok",
        "audio_packets_non_audio",
        "cmd_vel_messages_rx",
        "force_rpi_messages_rx",
        "cam_stable_messages_rx",
        "cam_angle_messages_rx",
        "audio_tx_messages_rx",
    ]

    for field in fields:
        if field in driver_df.columns:
            summary[field] = last.get(field)

    return summary


def compute_driver_rates(driver_summary: dict, duration_sec) -> dict:
    if not driver_summary or duration_sec is None or duration_sec <= 0:
        return {}

    def num(key: str) -> float:
        try:
            return float(driver_summary.get(key, 0))
        except Exception:
            return 0.0

    udp_rx_total = num("udp_packets_rx_total")
    udp_unknown = num("udp_unknown_packets")

    result = {
        "udp_rx_rate_hz": udp_rx_total / duration_sec,
        "telemetry_rx_rate_hz_driver": num("telemetry_packets_rx") / duration_sec,
        "video_rx_rate_hz_driver": num("video_packets_ok") / duration_sec,
        "audio_rx_rate_hz_driver": num("audio_packets_ok") / duration_sec,
        "udp_unknown_ratio": None if udp_rx_total == 0 else udp_unknown / udp_rx_total,
        "udp_timeout_rate_hz": num("udp_socket_timeouts") / duration_sec,
        "udp_recv_error_rate_hz": num("udp_recv_errors") / duration_sec,
        "udp_command_send_error_rate_hz": num("udp_command_send_errors") / duration_sec,
    }

    return result


def make_plot_dirs(output_dir: Path, session_name: str) -> Path:
    plot_dir = output_dir / "plots" / session_name
    plot_dir.mkdir(parents=True, exist_ok=True)
    return plot_dir


def numeric_series(df: pd.DataFrame, column: str) -> pd.Series:
    if df.empty or column not in df.columns:
        return pd.Series(dtype="float64")
    return pd.to_numeric(df[column], errors="coerce").dropna()


def plot_dt_timeline(df: pd.DataFrame, stream_name: str, plot_dir: Path) -> None:
    if df.empty or "elapsed_sec" not in df.columns or "dt_since_previous_sec" not in df.columns:
        return

    elapsed = pd.to_numeric(df["elapsed_sec"], errors="coerce")
    dt = pd.to_numeric(df["dt_since_previous_sec"], errors="coerce")

    valid = elapsed.notna() & dt.notna()
    if valid.sum() == 0:
        return

    plt.figure(figsize=(10, 4))
    plt.plot(elapsed[valid], dt[valid], linewidth=0.8)
    plt.xlabel("Elapsed time (s)")
    plt.ylabel("Inter-message interval dt (s)")
    plt.title(f"{stream_name} timing timeline")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(plot_dir / f"{stream_name}_dt_timeline.png", dpi=300)
    plt.close()


def plot_dt_histogram(df: pd.DataFrame, stream_name: str, plot_dir: Path) -> None:
    dt = numeric_series(df, "dt_since_previous_sec")
    if len(dt) == 0:
        return

    plt.figure(figsize=(8, 4))
    plt.hist(dt, bins=60)
    plt.xlabel("Inter-message interval dt (s)")
    plt.ylabel("Count")
    plt.title(f"{stream_name} dt distribution")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(plot_dir / f"{stream_name}_dt_histogram.png", dpi=300)
    plt.close()


def plot_rate_over_time(df: pd.DataFrame, stream_name: str, plot_dir: Path, window_sec: float = 10.0) -> None:
    if df.empty or "elapsed_sec" not in df.columns:
        return

    elapsed = pd.to_numeric(df["elapsed_sec"], errors="coerce").dropna()
    if len(elapsed) == 0:
        return

    temp = pd.DataFrame({"elapsed_sec": elapsed})
    temp["bin"] = (temp["elapsed_sec"] // window_sec).astype(int)
    grouped = temp.groupby("bin").size().reset_index(name="count")
    grouped["time_sec"] = grouped["bin"] * window_sec
    grouped["rate_hz"] = grouped["count"] / window_sec

    plt.figure(figsize=(10, 4))
    plt.plot(grouped["time_sec"], grouped["rate_hz"], marker="o", markersize=2, linewidth=0.8)
    plt.xlabel("Elapsed time (s)")
    plt.ylabel("Rate (Hz)")
    plt.title(f"{stream_name} rate over time ({window_sec:.0f} s windows)")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(plot_dir / f"{stream_name}_rate_over_time.png", dpi=300)
    plt.close()


def plot_variable_from_typed(typed_df: pd.DataFrame, variable: str, plot_dir: Path) -> None:
    if typed_df.empty or "elapsed_sec" not in typed_df.columns or variable not in typed_df.columns:
        return

    elapsed = pd.to_numeric(typed_df["elapsed_sec"], errors="coerce")
    values = pd.to_numeric(typed_df[variable], errors="coerce")

    valid = elapsed.notna() & values.notna()
    if valid.sum() == 0:
        return

    plt.figure(figsize=(10, 4))
    plt.plot(elapsed[valid], values[valid], linewidth=0.8)
    plt.xlabel("Elapsed time (s)")
    plt.ylabel(variable)
    plt.title(f"{variable} over time")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(plot_dir / f"{variable}_over_time.png", dpi=300)
    plt.close()


def generate_plots(
    plot_dir: Path,
    control_df: pd.DataFrame,
    telemetry_df: pd.DataFrame,
    video_df: pd.DataFrame,
    audio_df: pd.DataFrame,
    typed_df: pd.DataFrame,
) -> None:
    streams = [
        ("cmd_vel", control_df),
        ("telemetry_json", telemetry_df),
        ("video", video_df),
    ]

    if not audio_df.empty and "direction" in audio_df.columns:
        streams.append(("audio_rx", audio_df[audio_df["direction"] == "rx"]))
        streams.append(("audio_tx", audio_df[audio_df["direction"] == "tx"]))

    for name, df in streams:
        plot_dt_timeline(df, name, plot_dir)
        plot_dt_histogram(df, name, plot_dir)
        plot_rate_over_time(df, name, plot_dir)

    for variable in [
        "voltage",
        "percentage",
        "current",
        "power",
        "pcb_temp",
        "cpu_temp",
        "robot_server_ping",
        "client_server_ping",
    ]:
        plot_variable_from_typed(typed_df, variable, plot_dir)


def write_text_report(
    output_path: Path,
    session_dir: Path,
    rate_rows: list[dict],
    variable_rows: list[dict],
    driver_summary: dict,
    driver_rate_summary: dict,
    plot_dir: Path,
) -> None:
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("GuardROS Log Analysis\n")
        f.write("=====================\n\n")
        f.write(f"Session directory:\n{session_dir}\n\n")
        f.write(f"Plot directory:\n{plot_dir}\n\n")

        f.write("1. Stream timing summary\n")
        f.write("------------------------\n")

        for row in rate_rows:
            f.write(f"\n[{row['stream']}]\n")
            f.write(f"  message_count            : {row['message_count']}\n")
            f.write(f"  duration_sec             : {row['duration_sec']}\n")
            f.write(f"  average_rate_hz          : {row['average_rate_hz']}\n")
            f.write(f"  expected_rate_hz         : {row['expected_rate_hz']}\n")
            f.write(f"  mean_dt_sec              : {row['mean_dt_sec']}\n")
            f.write(f"  median_dt_sec            : {row['median_dt_sec']}\n")
            f.write(f"  std_dt_sec               : {row['std_dt_sec']}\n")
            f.write(f"  min_dt_sec               : {row['min_dt_sec']}\n")
            f.write(f"  max_dt_sec               : {row['max_dt_sec']}\n")
            f.write(f"  p95_dt_sec               : {row['p95_dt_sec']}\n")
            f.write(f"  p99_dt_sec               : {row['p99_dt_sec']}\n")
            f.write(f"  gaps_above_2x_expected   : {row['gaps_above_2x_expected']}\n")
            f.write(f"  gaps_above_0_2_sec       : {row['gaps_above_0_2_sec']}\n")
            f.write(f"  stability_label          : {row['stability_label']}\n")

        f.write("\n\n2. Battery, temperature, and ping summary\n")
        f.write("-----------------------------------------\n")

        for row in variable_rows:
            f.write(f"\n[{row['variable']}]\n")
            f.write(f"  count  : {row['count']}\n")
            f.write(f"  mean   : {row['mean']}\n")
            f.write(f"  median : {row['median']}\n")
            f.write(f"  std    : {row['std']}\n")
            f.write(f"  min    : {row['min']}\n")
            f.write(f"  max    : {row['max']}\n")
            f.write(f"  p95    : {row['p95']}\n")
            f.write(f"  p99    : {row['p99']}\n")

        f.write("\n\n3. Final driver diagnostic counters\n")
        f.write("-----------------------------------\n")
        for key, value in driver_summary.items():
            f.write(f"{key}: {value}\n")

        f.write("\n\n4. Driver-level derived rates\n")
        f.write("-----------------------------\n")
        for key, value in driver_rate_summary.items():
            f.write(f"{key}: {value}\n")


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 experiments/analysis/analyze_guardros_logs.py <session_folder>")
        sys.exit(1)

    session_dir = Path(sys.argv[1]).expanduser().resolve()

    if not session_dir.exists():
        print(f"[ERROR] Session folder does not exist: {session_dir}")
        sys.exit(1)

    print(f"[INFO] Analyzing session: {session_dir}")

    output_dir = Path("experiments/analysis/results").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    session_name = session_dir.name
    plot_dir = make_plot_dirs(output_dir, session_name)

    control_df = read_csv_safe(session_dir / "control_log.csv")
    telemetry_df = read_csv_safe(session_dir / "telemetry_log.csv")
    video_df = read_csv_safe(session_dir / "video_log.csv")
    audio_df = read_csv_safe(session_dir / "audio_log.csv")
    typed_df = read_csv_safe(session_dir / "typed_telemetry_log.csv")

    driver_diag_df = read_jsonl_safe(session_dir / "diagnostics_driver_log.jsonl")

    rate_rows = [
        basic_rate_stats(control_df, "cmd_vel"),
        basic_rate_stats(telemetry_df, "telemetry_json"),
        basic_rate_stats(video_df, "video"),
    ]

    if not audio_df.empty and "direction" in audio_df.columns:
        rate_rows.append(basic_rate_stats(audio_df[audio_df["direction"] == "rx"], "audio_rx"))
        rate_rows.append(basic_rate_stats(audio_df[audio_df["direction"] == "tx"], "audio_tx"))
    else:
        rate_rows.append(basic_rate_stats(pd.DataFrame(), "audio_rx"))
        rate_rows.append(basic_rate_stats(pd.DataFrame(), "audio_tx"))

    variable_rows = summarize_variables(typed_df)
    driver_summary = summarize_driver_diagnostics(driver_diag_df)

    session_duration = rate_rows[0].get("duration_sec") if rate_rows else None
    driver_rate_summary = compute_driver_rates(driver_summary, session_duration)

    rate_summary_df = pd.DataFrame(rate_rows)
    variable_summary_df = pd.DataFrame(variable_rows)
    driver_summary_df = pd.DataFrame(
        [{"metric": key, "value": value} for key, value in driver_summary.items()]
    )
    driver_rate_summary_df = pd.DataFrame(
        [{"metric": key, "value": value} for key, value in driver_rate_summary.items()]
    )

    generate_plots(
        plot_dir,
        control_df,
        telemetry_df,
        video_df,
        audio_df,
        typed_df,
    )

    rate_summary_path = output_dir / f"{session_name}_rate_summary.csv"
    variable_summary_path = output_dir / f"{session_name}_variable_summary.csv"
    driver_summary_path = output_dir / f"{session_name}_driver_summary.csv"
    driver_rate_summary_path = output_dir / f"{session_name}_driver_rate_summary.csv"
    text_report_path = output_dir / f"{session_name}_report.txt"

    rate_summary_df.to_csv(rate_summary_path, index=False)
    variable_summary_df.to_csv(variable_summary_path, index=False)
    driver_summary_df.to_csv(driver_summary_path, index=False)
    driver_rate_summary_df.to_csv(driver_rate_summary_path, index=False)

    write_text_report(
        text_report_path,
        session_dir,
        rate_rows,
        variable_rows,
        driver_summary,
        driver_rate_summary,
        plot_dir,
    )

    print("[INFO] Analysis complete.")
    print(f"[INFO] Rate summary        : {rate_summary_path}")
    print(f"[INFO] Variable summary    : {variable_summary_path}")
    print(f"[INFO] Driver summary      : {driver_summary_path}")
    print(f"[INFO] Driver rate summary : {driver_rate_summary_path}")
    print(f"[INFO] Text report         : {text_report_path}")
    print(f"[INFO] Plots               : {plot_dir}")


if __name__ == "__main__":
    main()