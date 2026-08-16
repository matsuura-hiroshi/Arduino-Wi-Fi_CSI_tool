#!/usr/bin/env python3
# csi_ctl.py ― csi_rx / csi_tx の実行時設定ツール
#
# 使い方:
#   python csi_ctl.py time                  # PC の現在時刻を ESP32 に設定
#   python csi_ctl.py SHOW                  # 設定表示
#   python csi_ctl.py FORMAT AMP            # 出力を振幅に
#   python csi_ctl.py MODE COMPACT
#   python csi_ctl.py OUTPUT BOTH
#   python csi_ctl.py TARGET 192.168.4.255
#   python csi_ctl.py CHANNEL 6
#   python csi_ctl.py SAVE                  # NVS 永続化
#   python csi_ctl.py -i 192.168.4.2 RATE 50   # 宛先指定（csi_tx 等）
#
# 既定宛先は softAP の 192.168.4.1:5006．

import socket
import sys
import time

ip, port = "192.168.4.1", 5006
args = sys.argv[1:]

if len(args) >= 2 and args[0] == "-i":
    ip = args[1]
    args = args[2:]

if not args:
    print(__doc__ or "usage: csi_ctl.py [-i ip] <COMMAND...>")
    sys.exit(1)

if args[0].lower() == "time":
    cmd = f"TIME {time.time():.6f}"
else:
    cmd = " ".join(args)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2.0)
s.sendto(cmd.encode(), (ip, port))
print(f"> {cmd}")
try:
    data, addr = s.recvfrom(512)
    print(f"< {data.decode().strip()}  (from {addr[0]})")
except socket.timeout:
    print("< (no reply)")
