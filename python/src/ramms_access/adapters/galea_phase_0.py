"""OpenBCI Galea — Phase 0: live signal monitor (every channel, nothing sent).

Shows every row BrainFlow streams from the board, labeled from the board
description: EMG, EOG and EEG as RMS (uV) in a band suited to each signal,
the remaining rows (aux, package number, timestamp, marker) as their latest
raw value. Use it to check fit and contact before calibrating phase 1: a
relaxed channel should sit low and steady, a gesture should make it jump.

Usage:
  ramms-access galea-p0 --ip 192.168.0.6

Requires: pip install "ramms-access[galea]"  (brainflow, numpy)
"""

from __future__ import annotations

import os
import time
import numpy as np
from brainflow.board_shim import BoardIds, BoardShim, BrainFlowInputParams
from brainflow.data_filter import DataFilter, DetrendOperations, FilterTypes

BUFFER_S = 2.0      # filtered history (long enough for the 0.5 Hz EOG band to settle)
WINDOW_S = 0.5      # RMS window at the end of the buffer
REFRESH_S = 0.2

# Signal types shown as band-limited RMS, in display order: (descr key, label, band Hz).
EXG = [
    ("emg_channels", "EMG", (20.0, 100.0)),
    ("eog_channels", "EOG", (0.5, 10.0)),
    ("eeg_channels", "EEG", (1.0, 40.0)),
]
# Everything else is shown as its latest raw value.
RAW = [
    ("accel_channels", "accel"), ("gyro_channels", "gyro"), ("ppg_channels", "ppg"),
    ("eda_channels", "eda"), ("temperature_channels", "temp"), ("battery_channel", "battery"),
    ("other_channels", "other"), ("package_num_channel", "package"),
    ("timestamp_channel", "timestamp"), ("marker_channel", "marker"),
]


def channel_table(descr: dict) -> list[tuple[int, str, str, tuple[float, float] | None]]:
    """(row, type, name, band) for every row in the board description. A row
    listed under several types is shown once,
    under the first type."""
    eeg_names = descr.get("eeg_names", "").split(",") if descr.get("eeg_names") else []
    table, seen = [], set()
    for key, kind, band in EXG:
        for i, row in enumerate(descr.get(key, [])):
            if row in seen:
                continue
            seen.add(row)
            name = eeg_names[i] if kind == "EEG" and i < len(eeg_names) else f"{kind}{i + 1}"
            table.append((row, kind, name, band))
    for key, kind in RAW:
        rows = descr.get(key, [])
        rows = rows if isinstance(rows, list) else [rows]
        for i, row in enumerate(rows):
            if row in seen:
                continue
            seen.add(row)
            table.append((row, kind, f"{kind}{i + 1}" if len(rows) > 1 else kind, None))
    return table


def band_rms(x: np.ndarray, fs: int, band: tuple[float, float], win_n: int) -> float:
    x = np.ascontiguousarray(x, dtype=np.float64)
    DataFilter.detrend(x, DetrendOperations.CONSTANT.value)
    DataFilter.perform_bandstop(x, fs, 60 - 2.0, 60 + 2.0, 4,
                                FilterTypes.BUTTERWORTH.value, 0.0)
    DataFilter.perform_bandpass(x, fs, band[0], band[1], 4, FilterTypes.BUTTERWORTH.value, 0.0)
    w = x[-win_n:]
    return float(np.sqrt((w * w).mean()))


def connect(ip: str) -> tuple[BoardShim, int]:
    params = BrainFlowInputParams()
    if ip:
        params.ip_address = ip
    board_id = BoardIds.GALEA_BOARD.value
    board = BoardShim(board_id, params)
    board.prepare_session()
    board.start_stream()
    return board, board_id


def monitor(ip: str = "") -> None:
    BoardShim.disable_board_logger()
    board, board_id = connect(ip)
    descr = BoardShim.get_board_descr(board_id)
    fs = descr["sampling_rate"]
    table = channel_table(descr)
    buf_n, win_n = int(BUFFER_S * fs), int(WINDOW_S * fs)

    os.system("")
    header = (f"[galea-p0] {descr.get('name', 'board')} @ {fs} Hz — signals: RMS uV over the "
              f"last {WINDOW_S}s in the band shown; others: latest value. Ctrl-C to quit.")
    print(header)
    drawn = 0
    try:
        while True:
            data = board.get_current_board_data(buf_n)
            if data.shape[1] < buf_n:
                print(f"\r[galea-p0] filling buffer {data.shape[1]}/{buf_n} samples", end="", flush=True)
                time.sleep(REFRESH_S)
                continue
            lines = [f"{'row':>4}  {'type':<9} {'name':<10} {'value':>12}  band"]
            for row, kind, name, band in table:
                if band is not None:
                    value = f"{band_rms(data[row], fs, band, win_n):12.1f}"
                    note = f"{band[0]:g}-{band[1]:g} Hz"
                else:
                    value, note = f"{data[row, -1]:12.3f}", ""
                lines.append(f"{row:>4}  {kind:<9} {name:<10} {value}  {note}")
            print(("\r\x1b[%dF" % drawn if drawn else "\r") + "\n".join(
                line + "\x1b[K" for line in lines), flush=True)
            drawn = len(lines)
            time.sleep(REFRESH_S)
    except KeyboardInterrupt:
        print()
    finally:
        try:
            board.stop_stream()
            board.release_session()
        except Exception:
            pass
