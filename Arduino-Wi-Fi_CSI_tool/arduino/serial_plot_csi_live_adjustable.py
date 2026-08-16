#!/usr/bin/env python3
# serial_plot_csi_live_adjustable.py
# ─ csi_tbeam（DISP CSV モード）のシリアル出力をリアルタイム描画する調整版 ─
# 上段：注目サブキャリアの振幅時系列／下段：時間×サブキャリアの振幅ヒートマップ．
#
# 元の serial_plot_csi_live.py に「調整つまみ」を追加したもの．
#   ・色レンジ vmin / vmax をスライダーで自由設定（ESP-IDF のヒートマップに合わせ込める）
#   ・表示窓の長さ window（何フレーム分を出すか）を可変
#   ・正規化モードを3種から選択（ここが見え方の本命）
#       RAW  : 生振幅 sqrt(I^2+R^2)．従来どおり．AGC ゲイン段差もそのまま映る
#       MEAN : フレームごとに「データSCの平均」で割って ref(=50) に正規化．
#              受信 AGC の段差（フレーム全体が一斉に明暗）を消し，SC 方向の形だけ残す
#       BASE : ベースラインからの差分 |amp - base| を表示．動いた所だけが光る
#              [Set baseline] で現在の窓平均を基準に固定（未設定なら窓の移動平均）
#   ・DC/パイロットSC のマスク ON/OFF（ESP-IDF 側が DC を消していない時の比較用）
#
# 準備:  pip install pyserial numpy matplotlib
# 使い方:
#   python serial_plot_csi_live_adjustable.py /dev/cu.usbserial-XXXX        # SC=44 から
#   python serial_plot_csi_live_adjustable.py COM5 15                       # 開始SC指定
#
# 前提: r 側が DISP CSV（CSI_DATA 行，FORMAT RAW）．シリアルモニタは閉じてから実行．
# 操作はすべてマウス．つまみの状態は各グラフのタイトルに英語表示される．

import sys
import collections
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider

PORT      = sys.argv[1] if len(sys.argv) > 1 else "COM5"
START_SC  = int(sys.argv[2]) if len(sys.argv) > 2 else 44
BAUD      = 921600
N_SC      = 64
MASK_IDX  = [0, 1, 2, 3, 32]      # DC・ガード・パイロット相当
MAX_BUFFER = 600                  # 内部リングバッファ（window スライダーの上限はこの範囲内）
REF_MEAN  = 50.0                  # MEAN モードの正規化目標値

ON_COLOR  = "#79d279"             # 緑（オン/有効）
OFF_COLOR = "#cccccc"             # 灰（オフ/無効）

state = {
    "sc":          START_SC,
    "auto_sc":     False,
    "metric":      "var",         # auto_sc の選択基準 var|amp
    "scale_fixed": False,         # False=色レンジおまかせ / True=vmin..vmax 固定
    "norm":        "raw",         # raw|mean|base
    "mask_dc":     True,
    "baseline":    None,          # BASE モードの基準ベクトル（N_SC,）
}

# ── シリアル接続 ──
import serial
ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"open {PORT} @ {BAUD}")
print("操作は画面下のボタン／スライダーをマウスで．")

hist = collections.deque(maxlen=MAX_BUFFER)

# ── 図とウィジェット ──
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 9.6))
fig.subplots_adjust(bottom=0.32, top=0.95, hspace=0.35)


def _btn(left, bottom, width, label, h=0.045):
    return Button(fig.add_axes([left, bottom, width, h]), label)


def _slider(bottom, label, vmin, vmax, vinit, step):
    ax = fig.add_axes([0.12, bottom, 0.78, 0.025])
    return Slider(ax, label, vmin, vmax, valinit=vinit, valstep=step)


# トグル系ボタン（1段目）
b_scale  = _btn(0.030, 0.205, 0.150, "")
b_norm   = _btn(0.190, 0.205, 0.150, "")
b_dc     = _btn(0.350, 0.205, 0.150, "")
b_autosc = _btn(0.510, 0.205, 0.150, "")
b_metric = _btn(0.670, 0.205, 0.150, "")
b_base   = _btn(0.830, 0.205, 0.140, "Set baseline")

# SC 手動増減（2段目・左の小ボタン）
b_scdec  = _btn(0.030, 0.255, 0.070, "SC -", h=0.038)
b_scinc  = _btn(0.105, 0.255, 0.070, "SC +", h=0.038)

# スライダー（vmin / vmax / window）
s_vmin = _slider(0.150, "vmin",   0,  300,  0.0,  5)
s_vmax = _slider(0.110, "vmax",   5,  500, 60.0,  5)
s_win  = _slider(0.070, "window", 20, MAX_BUFFER, 100, 10)


def _set(btn, text, on):
    btn.label.set_text(text)
    c = ON_COLOR if on else OFF_COLOR
    btn.color = c
    btn.ax.set_facecolor(c)


def refresh_buttons():
    _set(b_scale, "Scale: FIX" if state["scale_fixed"] else "Scale: AUTO",
         state["scale_fixed"])
    norm_lbl = {"raw": "Norm: RAW", "mean": "Norm: MEAN", "base": "Norm: BASE"}
    _set(b_norm, norm_lbl[state["norm"]], state["norm"] != "raw")
    _set(b_dc, "DC mask: ON" if state["mask_dc"] else "DC mask: OFF", state["mask_dc"])
    _set(b_autosc, "SC: AUTO" if state["auto_sc"] else "SC: MANUAL", state["auto_sc"])
    _set(b_metric, "Metric: VAR" if state["metric"] == "var" else "Metric: AMP",
         state["metric"] == "var")
    fig.canvas.draw_idle()


def cb_scale(_):  state["scale_fixed"] = not state["scale_fixed"]; refresh_buttons()
def cb_dc(_):     state["mask_dc"] = not state["mask_dc"]; refresh_buttons()
def cb_autosc(_): state["auto_sc"] = not state["auto_sc"]; refresh_buttons()
def cb_metric(_): state["metric"] = "amp" if state["metric"] == "var" else "var"; refresh_buttons()


def cb_norm(_):
    order = {"raw": "mean", "mean": "base", "base": "raw"}
    state["norm"] = order[state["norm"]]
    refresh_buttons()


def cb_base(_):
    # 現在の窓のSC方向平均をベースラインとして固定
    if len(hist) > 2:
        win = int(s_win.val)
        df = np.asarray(hist, dtype=np.float32)[-win:]
        state["baseline"] = df.mean(axis=0)
        print(f"[baseline] captured over {df.shape[0]} frames")


def cb_sc_dec(_): state["auto_sc"] = False; state["sc"] = max(state["sc"] - 1, 0); refresh_buttons()
def cb_sc_inc(_): state["auto_sc"] = False; state["sc"] = min(state["sc"] + 1, N_SC - 1); refresh_buttons()


b_scale.on_clicked(cb_scale)
b_norm.on_clicked(cb_norm)
b_dc.on_clicked(cb_dc)
b_autosc.on_clicked(cb_autosc)
b_metric.on_clicked(cb_metric)
b_base.on_clicked(cb_base)
b_scdec.on_clicked(cb_sc_dec)
b_scinc.on_clicked(cb_sc_inc)
refresh_buttons()


# ── データ処理 ──
def parse_line(line: str):
    if "CSI_DATA" not in line or "[" not in line:
        return None
    try:
        arr = line.split("[", 1)[1].split("]", 1)[0].split()
        vals = [int(v) for v in arr]
    except (ValueError, IndexError):
        return None
    if len(vals) < 2:
        return None
    imag = vals[0::2]
    real = vals[1::2]
    n = min(len(imag), len(real), N_SC)
    amp = np.sqrt(np.array(imag[:n], float) ** 2 + np.array(real[:n], float) ** 2)
    if n < N_SC:
        amp = np.pad(amp, (0, N_SC - n))
    return amp  # 生振幅のまま蓄える（正規化は表示時に行う）


def compute_display(df_raw):
    """蓄えた生振幅 (frames, N_SC) を現在の正規化モードで表示用に変換"""
    df = df_raw.copy()
    data_cols = [i for i in range(df.shape[1]) if i not in MASK_IDX]
    mode = state["norm"]
    if mode == "mean":
        m = df[:, data_cols].mean(axis=1, keepdims=True)
        m[m < 1e-6] = 1e-6
        df = df / m * REF_MEAN
    elif mode == "base":
        base = state["baseline"] if state["baseline"] is not None else df.mean(axis=0)
        df = np.abs(df - base)
    if state["mask_dc"]:
        for i in MASK_IDX:
            if i < df.shape[1]:
                df[:, i] = 0.0
    return df


def best_subcarrier(df):
    score = df.var(axis=0) if state["metric"] == "var" else df.mean(axis=0)
    if state["mask_dc"]:
        for i in MASK_IDX:
            if i < len(score):
                score[i] = -1.0
    return int(np.argmax(score))


def redraw():
    win = int(s_win.val)
    df_raw = np.asarray(hist, dtype=np.float32)[-win:]
    df = compute_display(df_raw)

    if state["auto_sc"]:
        state["sc"] = best_subcarrier(df)
    sc = min(state["sc"], df.shape[1] - 1)

    lo, hi = float(s_vmin.val), float(s_vmax.val)
    if hi <= lo:
        hi = lo + 1.0

    # 上段：時系列
    ax1.cla()
    ax1.plot(df[:, sc], color="r")
    mode_sc = "AUTO" if state["auto_sc"] else "MANUAL"
    scale_s = f"FIX[{lo:.0f}..{hi:.0f}]" if state["scale_fixed"] else "AUTO"
    ax1.set_title(
        f"Amplitude Time Series  SC{sc}  "
        f"[Norm:{state['norm'].upper()} / SC:{mode_sc}/{state['metric'].upper()}]  "
        f"Scale:{scale_s}  win={win}"
    )
    if state["scale_fixed"]:
        ax1.set_ylim(lo, hi)
    else:
        ax1.set_ylim(0, max(df.max() * 1.1, 1))

    # 下段：ヒートマップ
    ax2.cla()
    if state["scale_fixed"]:
        ax2.imshow(df.T, aspect="auto", cmap="jet", origin="lower", vmin=lo, vmax=hi)
    else:
        ax2.imshow(df.T, aspect="auto", cmap="jet", origin="lower")
    ax2.set_title(f"CSI Amplitude Heatmap ({N_SC} Subcarriers)  Norm:{state['norm'].upper()}")
    ax2.set_xlabel("Time")
    ax2.set_ylabel("Subcarrier")
    ax2.axhline(sc, color="white", linewidth=0.8, alpha=0.6)

    fig.canvas.flush_events()
    plt.pause(0.001)


# ── メインループ ──
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
