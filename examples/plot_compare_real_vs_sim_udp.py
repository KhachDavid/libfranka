#!/usr/bin/env python3
import argparse
import json
import queue
import socket
import threading
import time
from typing import List

import matplotlib.pyplot as plt
import numpy as np


def listen(port: int, out_q: queue.Queue, name: str):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.2)
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            msg = json.loads(data.decode("utf-8"))
            out_q.put((name, msg))
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="Compare real vs sim joint positions")
    parser.add_argument("--real-port", type=int, default=5601, help="UDP port for real robot stream")
    parser.add_argument("--sim-port", type=int, default=5602, help="UDP port for sim stream")
    parser.add_argument("--hz", type=float, default=100.0, help="Display sample rate")
    parser.add_argument("--ms", type=float, default=1.0, help="Time grid in milliseconds (1.0 = libfranka step)")
    parser.add_argument("--align-auto", action="store_true", help="Auto-estimate real-vs-sim lag by cross-correlation")
    parser.add_argument("--align-joint", type=int, default=5, help="Joint index [0..6] to use for lag estimation")
    parser.add_argument("--offset-ms", type=float, default=0.0, help="Manual offset to apply to REAL stream in ms (positive shifts real earlier)")
    args = parser.parse_args()

    q = queue.Queue()
    if args.real_port == args.sim_port:
        # Single listener on one port; streams must self-identify via 'src' field
        t = threading.Thread(target=listen, args=(args.real_port, q, "both"), daemon=True)
        t.start()
    else:
        tr = threading.Thread(target=listen, args=(args.real_port, q, "real"), daemon=True)
        ts = threading.Thread(target=listen, args=(args.sim_port, q, "sim"), daemon=True)
        tr.start(); ts.start()

    plt.ion()
    fig, axes = plt.subplots(7, 1, figsize=(8, 10), sharex=True)
    # Historical buffers
    times_real: List[float] = []       # RobotState::time
    times_sim: List[float] = []
    wall_real: List[float] = []        # publisher wall-time since process start
    wall_sim: List[float] = []
    step_real: List[int] = []          # discrete step index from publisher
    step_sim: List[int] = []
    data_real: List[List[float]] = [[] for _ in range(7)]
    data_sim: List[List[float]] = [[] for _ in range(7)]
    lines = []
    for j in range(7):
        l1, = axes[j].plot([], [], label=f"real q{j}")
        l2, = axes[j].plot([], [], label=f"sim q{j}")
        axes[j].legend(loc="upper right")
        axes[j].set_ylabel(f"q{j} [rad]")
        lines.append((l1, l2))
    axes[-1].set_xlabel("t [s]")

    latest = {"real": None, "sim": None}
    base_t = {"real": None, "sim": None}
    sample_dt = 1.0 / args.hz
    next_sample = time.time()

    while plt.fignum_exists(fig.number):
        # Drain incoming
        while True:
            try:
                name, msg = q.get_nowait()
                # Allow either publisher to self-identify via 'src' field
                if isinstance(msg, dict) and 'src' in msg:
                    if msg['src'] == 'real':
                        name = 'real'
                    elif msg['src'] == 'sim':
                        name = 'sim'
                elif name == 'both':
                    # If no src, leave as 'both'; will be ignored until identified
                    pass
                latest[name] = msg
            except queue.Empty:
                break

        now = time.time()
        if now < next_sample:
            plt.pause(0.005)
            continue
        next_sample += sample_dt

        if latest["real"] is None and latest["sim"] is None:
            plt.pause(0.005)
            continue

        # Append latest samples to buffers (guard if one stream missing)
        if latest["real"] is not None:
            tr = float(latest["real"].get("t", 0.0))
            if base_t["real"] is None: base_t["real"] = tr
            tr -= base_t["real"]
            times_real.append(tr)
            if 'w' in latest["real"]:
                wall_real.append(float(latest["real"].get('w', 0.0)))
                step_real.append(int(latest["real"].get('k', len(step_real))))
        if latest["sim"] is not None:
            ts = float(latest["sim"].get("t", 0.0))
            if base_t["sim"] is None: base_t["sim"] = ts
            ts -= base_t["sim"]
            times_sim.append(ts)
            if 'w' in latest["sim"]:
                wall_sim.append(float(latest["sim"].get('w', 0.0)))
                step_sim.append(int(latest["sim"].get('k', len(step_sim))))
        qr = latest["real"].get("q") if latest["real"] is not None else None
        qs = latest["sim"].get("q") if latest["sim"] is not None else None
        for j in range(7):
            if qr is not None:
                data_real[j].append(float(qr[j]))
            if qs is not None:
                data_sim[j].append(float(qs[j]))

        # Build a libfranka-style time grid (quantized in ms) using UNION of ranges by step index when available.
        t_max_all = 0.0
        if step_real and step_sim:
            # Align by discrete step index
            k_max = max(step_real[-1], step_sim[-1])
            dt = max(args.ms, 0.1) / 1000.0
            t_grid = np.arange(0, (k_max + 1) * dt, dt)
            tr_arr = np.array(times_real) if len(times_real) else None
            ts_arr = np.array(times_sim) if len(times_sim) else None
            kr = np.array(step_real) if step_real else None
            ks = np.array(step_sim) if step_sim else None
            # Interpolate all joints first
            yr_all = []
            ys_all = []
            for j in range(7):
                if kr is not None and len(kr) >= 2:
                    yr = np.array(data_real[j])
                    yr_i = np.interp(t_grid, kr * dt, yr, left=np.nan, right=np.nan)
                else:
                    yr_i = np.full_like(t_grid, np.nan)
                if ks is not None and len(ks) >= 2:
                    ys = np.array(data_sim[j])
                    ys_i = np.interp(t_grid, ks * dt, ys, left=np.nan, right=np.nan)
                else:
                    ys_i = np.full_like(t_grid, np.nan)
                yr_all.append(yr_i)
                ys_all.append(ys_i)

            # Optional alignment
            offset_k = 0
            if args.align_auto and 0 <= args.align_joint < 7:
                a = yr_all[args.align_joint]
                b = ys_all[args.align_joint]
                mask = ~(np.isnan(a) | np.isnan(b))
                if np.count_nonzero(mask) > 10:
                    aa = a[mask] - np.nanmean(a[mask])
                    bb = b[mask] - np.nanmean(b[mask])
                    cc = np.correlate(aa, bb, mode='full')
                    lag = np.argmax(cc) - (len(bb) - 1)
                    offset_k = -lag  # shift REAL forward if it lags (positive lag)
            if abs(args.offset_ms) > 0.0:
                offset_k += int(round(args.offset_ms / (dt * 1000.0)))

            def shift(arr, k):
                if k == 0:
                    return arr
                out = np.full_like(arr, np.nan)
                if k > 0:
                    out[:-k] = arr[k:]
                else:
                    out[-k:] = arr[:k]
                return out

            for j in range(7):
                yr_i = shift(yr_all[j], offset_k)
                ys_i = ys_all[j]
                lines[j][0].set_data(t_grid, yr_i)
                lines[j][1].set_data(t_grid, ys_i)
            for ax in axes:
                ax.relim(); ax.autoscale_view()
            plt.pause(0.001)
            continue

        if times_real: t_max_all = max(t_max_all, max(times_real))
        if times_sim:  t_max_all = max(t_max_all, max(times_sim))
        if t_max_all > 0:
            dt = max(args.ms, 0.1) / 1000.0
            t_grid = np.arange(0.0, t_max_all, dt)
            tr_arr = np.array(times_real) if len(times_real) else None
            ts_arr = np.array(times_sim) if len(times_sim) else None
            yr_all = []
            ys_all = []
            for j in range(7):
                # Real stream
                if tr_arr is not None and len(tr_arr) >= 2:
                    yr = np.array(data_real[j])
                    yr_i = np.interp(t_grid, tr_arr, yr, left=np.nan, right=np.nan)
                elif tr_arr is not None and len(tr_arr) == 1:
                    yr_i = np.full_like(t_grid, np.nan); yr_i[0] = data_real[j][0]
                else:
                    yr_i = np.full_like(t_grid, np.nan)
                yr_all.append(yr_i)
                # Sim stream
                if ts_arr is not None and len(ts_arr) >= 2:
                    ys = np.array(data_sim[j])
                    ys_i = np.interp(t_grid, ts_arr, ys, left=np.nan, right=np.nan)
                elif ts_arr is not None and len(ts_arr) == 1:
                    ys_i = np.full_like(t_grid, np.nan); ys_i[0] = data_sim[j][0]
                else:
                    ys_i = np.full_like(t_grid, np.nan)
                ys_all.append(ys_i)

            # Optional alignment
            offset_k = 0
            if args.align_auto and 0 <= args.align_joint < 7:
                a = yr_all[args.align_joint]
                b = ys_all[args.align_joint]
                mask = ~(np.isnan(a) | np.isnan(b))
                if np.count_nonzero(mask) > 10:
                    aa = a[mask] - np.nanmean(a[mask])
                    bb = b[mask] - np.nanmean(b[mask])
                    cc = np.correlate(aa, bb, mode='full')
                    lag = np.argmax(cc) - (len(bb) - 1)
                    offset_k = -lag
            if abs(args.offset_ms) > 0.0:
                offset_k += int(round(args.offset_ms / (dt * 1000.0)))

            def shift(arr, k):
                if k == 0:
                    return arr
                out = np.full_like(arr, np.nan)
                if k > 0:
                    out[:-k] = arr[k:]
                else:
                    out[-k:] = arr[:k]
                return out

            for j in range(7):
                yr_i = shift(yr_all[j], offset_k)
                ys_i = ys_all[j]
                lines[j][0].set_data(t_grid, yr_i)
                lines[j][1].set_data(t_grid, ys_i)

            for ax in axes:
                ax.relim(); ax.autoscale_view()
        plt.pause(0.001)


if __name__ == "__main__":
    main()


