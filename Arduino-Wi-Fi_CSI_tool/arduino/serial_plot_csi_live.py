#!/usr/bin/env python3
# serial_plot_csi_live.py ― csi_tbeam（DISP CSV モード）のシリアル出力を
# リアルタイム描画する．上段：注目サブキャリアの振幅時系列，
# 下段：時間×サブキャリアの振幅ヒートマップ．
#
# 準備:  pip install pyserial numpy matplotlib
# 使い方:
#   python serial_plot_csi_live.py /dev/cu.usbserial-XXXX        # SC=44 から開始
#   python serial_plot_csi_live.py /dev/cu.usbserial-XXXX 15     # 開始SCを指定
#
# 前提: r 側が DISP CSV（CSI_DATA 行）
# 注意: Arduino IDE のシリアルモニタを閉じてから実行する（ポート競合）
#
# ───────────────────────────────────────────────────────────────
#  操作はすべて画面下のボタンをマウスでクリック（キー操作不要）
#  ボタンの文字は現在の状態を表す．緑=有効/オン，灰=無効/オフ：
#   Scale: AUTO / FIX        スケール オート⇔固定
#   SC: MANUAL / AUTO        上段サブキャリア 手動⇔自動選択
#   Metric: VAR / AMP        自動選択の基準 変動⇔強度
#   [ SC - ] [ SC + ]        手動時にサブキャリア番号を増減
#   [ vmax - ] [ vmax + ]    固定スケール上限を増減
#  状態は上段グラフのタイトルにも英語で常時表示される
# ───────────────────────────────────────────────────────────────

import sys
import collections
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import serial

PORT       = sys.argv[1] if len(sys.argv) > 1 else "COM5"
START_SC   = int(sys.argv[2]) if len(sys.argv) > 2 else 44
BAUD       = 921600
MAX_HIST   = 100
N_SC       = 64
MASK_DC    = True
MASK_IDX   = [0, 1, 2, 3, 32]

ON_COLOR  = "#79d279"   # 緑（オン/有効）
OFF_COLOR = "#cccccc"   # 灰（オフ/無効）

state = {
    "sc":          START_SC,
    "auto_sc":     False,
    "metric":      "var",
    "scale_fixed": False,
    "vmax":        60.0,
}

ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"open {PORT} @ {BAUD}")
print("操作は画面下のボタンをマウスでクリックしてください")

hist = collections.deque(maxlen=MAX_HIST)

plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8.5))
fig.subplots_adjust(bottom=0.16, hspace=0.35)


def _btn(left, width, label):
    axb = fig.add_axes([left, 0.04, width, 0.06])
    return Button(axb, label)


b_scale  = _btn(0.02, 0.17, "")
b_autosc = _btn(0.21, 0.17, "")
b_metric = _btn(0.40, 0.17, "")
b_scdec  = _btn(0.59, 0.08, "SC -")
b_scinc  = _btn(0.68, 0.08, "SC +")
b_vdec   = _btn(0.79, 0.08, "vmax -")
b_vinc   = _btn(0.88, 0.08, "vmax +")


def _set(btn, text, on):
    btn.label.set_text(text)
    c = ON_COLOR if on else OFF_COLOR
    btn.color = c
    btn.ax.set_facecolor(c)


def refresh_button_labels():
    # FIX が有効なときは緑，AUTO（スケールおまかせ）なら灰
    _set(b_scale,  "Scale: FIX" if state["scale_fixed"] else "Scale: AUTO",
         state["scale_fixed"])
    # 自動選択が有効なら緑
    _set(b_autosc, "SC: AUTO" if state["auto_sc"] else "SC: MANUAL",
         state["auto_sc"])
    # var を緑，amp を灰（どちらでも良いが色で区別できるように）
    _set(b_metric, "Metric: VAR" if state["metric"] == "var" else "Metric: AMP",
         state["metric"] == "var")
    fig.canvas.draw_idle()


def cb_scale(_):
    state["scale_fixed"] = not state["scale_fixed"]; refresh_button_labels()


def cb_autosc(_):
    state["auto_sc"] = not state["auto_sc"]; refresh_button_labels()


def cb_metric(_):
    state["metric"] = "amp" if state["metric"] == "var" else "var"; refresh_button_labels()


def cb_sc_dec(_):
    state["auto_sc"] = False; state["sc"] = max(state["sc"] - 1, 0); refresh_button_labels()


def cb_sc_inc(_):
    state["auto_sc"] = False; state["sc"] = min(state["sc"] + 1, N_SC - 1); refresh_button_labels()


def cb_vmax_dec(_):
    state["vmax"] = max(state["vmax"] - 10, 10)


def cb_vmax_inc(_):
    state["vmax"] = min(state["vmax"] + 10, 500)


b_scale.on_clicked(cb_scale)
b_autosc.on_clicked(cb_autosc)
b_metric.on_clicked(cb_metric)
b_scdec.on_clicked(cb_sc_dec)
b_scinc.on_clicked(cb_sc_inc)
b_vdec.on_clicked(cb_vmax_dec)
b_vinc.on_clicked(cb_vmax_inc)

refresh_button_labels()


def parse_line(line: str):
    if "CSI_DATA" not in line or "[" not in line:
        return None
    try:
        arr = line.split("[", 1)[1].split("]", 1)[0].split()
        vals = [int(v) for v in arr]
    except ValueError:
        return None
    if len(vals) < 2:
        return None
    imag = vals[0::2]
    real = vals[1::2]
    n = min(len(imag), len(real), N_SC)
    amp = np.sqrt(np.array(imag[:n], float) ** 2 + np.array(real[:n], float) ** 2)
    if n < N_SC:
        amp = np.pad(amp, (0, N_SC - n))
    if MASK_DC:
        for i in MASK_IDX:
            if i < N_SC:
                amp[i] = 0
    return amp


def best_subcarrier(df):
    if state["metric"] == "var":
        score = df.var(axis=0)
    else:
        score = df.mean(axis=0)
    if MASK_DC:
        for i in MASK_IDX:
            if i < len(score):
                score[i] = -1.0
    return int(np.argmax(score))


def redraw():
    df = np.asarray(hist, dtype=np.float32)

    if state["auto_sc"]:
        state["sc"] = best_subcarrier(df)
    sc = min(state["sc"], df.shape[1] - 1)

    ax1.cla()
    ax1.plot(df[:, sc], color="r")
    mode_sc = "AUTO" if state["auto_sc"] else "MANUAL"
    scale_s = f"FIX(vmax={state['vmax']:.0f})" if state["scale_fixed"] else "AUTO"
    ax1.set_title(f"Amplitude Time Series  SC{sc}  [SC:{mode_sc} / {state['metric'].upper()}]  Scale:{scale_s}")
    if state["scale_fixed"]:
        ax1.set_ylim(0, state["vmax"])
    else:
        ax1.set_ylim(0, max(df.max() * 1.1, 1))

    ax2.cla()
    if state["scale_fixed"]:
        ax2.imshow(df.T, aspect="auto", cmap="jet", origin="lower",
                   vmin=0, vmax=state["vmax"])
    else:
        ax2.imshow(df.T, aspect="auto", cmap="jet", origin="lower")
    ax2.set_title(f"CSI Amplitude Heatmap ({N_SC} Subcarriers)")
    ax2.set_xlabel("Time")
    ax2.set_ylabel("Subcarrier")
    ax2.axhline(sc, color="white", linewidth=0.8, alpha=0.6)

    fig.canvas.flush_events()
    plt.pause(0.001)


count = 0
while True:
    raw = ser.readline()
    if not raw:
        continue
    line = raw.decode(errors="ignore").strip()
    if line.startswith("[STAT"):
        print(line)
        continue
    amp = parse_line(line)
    if amp is None:
        continue
    hist.append(amp)
    count += 1
    if count % 5 == 0 and len(hist) > 2:
        redraw()
