# 1. Arduino Wi-Fi CSI Tool

This software was designed for Arduino IDE-based Wi-Fi CSI sensing. It is intended for university education and simple experiments. Further refinement would be necessary for commercial use.

---

## 1.1 Features

Settings such as Wi-Fi channel and transmit power can be changed dynamically via serial or UDP commands after board startup. Settings are automatically saved to NVS, so they persist after board restart.

Three Arduino sketches are provided: a receiver, a transmitter, and a single-sketch demonstration version for T-Beam that can act as either role. Two Python plotting scripts and a command-line configuration tool are also included.

---

## 1.2 Tested and Working

The Arduino IDE version has been verified on T-Beam (LilyGO), ESP32-WROOM, and ESP32-S3.

Matsuura, partner companies engaged in collaborative research, and students have each independently verified and are operating it for their respective research purposes.

The ESP-IDF version is included but has not been tested on actual hardware.

---

## 1.3 Files

Everything included in this repository is listed below. There are no other files.

```
Arduino-Wi-Fi_CSI_tool/
├── csi_ctl.py                        Configuration tool (run from this folder)
│
├── arduino/                          Arduino IDE version
│   ├── csi_rx_arduino/
│   │   └── csi_rx_arduino.ino        Receiver: acts as softAP, collects CSI
│   ├── csi_tx_arduino/
│   │   └── csi_tx_arduino.ino        Transmitter: connects as STA, sends packets
│   ├── csi_tbeam/
│   │   └── csi_tbeam.ino             T-Beam demo: one sketch, either role
│   ├── serial_plot_csi_live.py       Live plotter (run from this folder)
│   └── serial_plot_csi_live_adjustable.py
│                                     Live plotter with normalisation controls
│
├── csi_rx/                           ESP-IDF version of the receiver (untested)
│   └── main/main.c
└── csi_tx/                           ESP-IDF version of the transmitter (untested)
    └── main/main.c
```

Note that `csi_ctl.py` sits at the top level while the plotting scripts sit under `arduino/`. Run each from its own folder.

---

## 1.4 Environment Setup

### Arduino IDE Version

Works with Arduino IDE + ESP32 board package 3.x or later. Install version 3.x of esp32 by Espressif Systems in the board manager, then open `arduino/csi_rx_arduino/csi_rx_arduino.ino` or `csi_tx_arduino.ino`. Select ESP32 Dev Module as the board and upload. Set the serial monitor to 921600 baud with LF (or CR+LF) line ending.

The receiver becomes an access point named `myssid` on channel 6, and the transmitter tries to connect to that same SSID. Both defaults can be changed with the SSID and PASS commands.

### T-Beam Demonstration Version

A single sketch for two T-Beam boards, in `arduino/csi_tbeam`. Select "T-Beam" or "ESP32 Dev Module" as the board.

Type `ROLE t` or `ROLE r` on the serial monitor to assign the role. The setting is saved to NVS and the board restarts into that role. `t` broadcasts via ESP-NOW at a fixed MCS0 rate; `r` receives CSI on the same channel. Both boards must be on the same channel, otherwise nothing arrives.

The receiver display can be toggled between `DISP STAT`, which prints a one-line summary every second (rx/s, loss rate, RSSI, amplitude of the selected subcarrier), and `DISP CSV`, which prints the full 26-column data.

Channels 1-13 are available under the JP regulatory setting. Channel 14 is 11b-only, so no CSI can be obtained there.

### ESP-IDF Version

The code should theoretically work with ESP-IDF 5.x, but it has not been tested on actual hardware.

---

## 1.5 Command List

### Receiver (csi_rx_arduino / csi_rx)

For Wi-Fi configuration, `SSID myssid` and `PASS mypassword` set AP authentication information; a password shorter than 8 characters produces an open AP because WPA2 cannot be used. `CHANNEL 6` sets channels 1-13, `BW 20` sets bandwidth to 20 or 40 MHz, `TXPOWER 20` sets power to 2-20 dBm. `PROTOCOL BGN` selects 11B, BG, or BGN, `MAXCONN 16` sets maximum connections from 1 to 16, and `HIDDEN 0` controls SSID hiding.

For CSI acquisition, `CSILLTF 1`, `CSIHT 1`, and `CSISTBC 1` control L-LTF, HT-LTF, and STBC HT-LTF2 activation. `CSIMERGE 1` controls LTF merging and `CSICHFILT 0` controls channel filtering. `CSISCALE 0` and `CSISHIFT 0` set manual scaling and shift amount from 0 to 15.

For output, `FORMAT RAW` selects RAW (raw I/Q), AMP (amplitude), or PHASE. `MODE FULL` selects FULL (26 columns) or COMPACT (5 columns). `OUTPUT UDP` selects UDP, SERIAL, or BOTH. `TARGET 192.168.4.255` sets the destination (ending in 255 for broadcast) and `PORT 5005` sets the destination port. `LLTF 1` limits the array to the first 128 values, and `FILTER 2C:BC:BB:83:1E:D8` outputs only the specified MAC (`FILTER OFF` to disable).

`TIME` sets the clock in unix seconds; `python csi_ctl.py time` sends the PC clock. Management commands are `SHOW`, `SAVE`, `RESET`, `REBOOT`, and `HELP`.

### Transmitter (csi_tx_arduino / csi_tx)

`SSID myssid` and `PASS mypassword` set the target AP and reconnect immediately. `TXPOWER 20` sets power to 2-20 dBm and `PROTOCOL BGN` selects 11B, BG, or BGN.

`RATE 100` sets the rate from 1 to 1000 Hz and `SIZE 8` sets the payload from 1 to 1400 bytes. `DSTIP 192.168.4.1` and `DSTPORT 5010` set the destination, which normally needs no change.

Management commands are `SHOW`, `SAVE`, `RESET`, `REBOOT`, and `HELP`.

### T-Beam Demo (csi_tbeam)

This sketch has its own command set, and several ranges differ from the transmitter above.

`ROLE t` and `ROLE r` assign the role, saving and restarting. `CHANNEL 6` sets channels 1-13 and `TXPOWER 20` sets power to 2-20 dBm; both must match between the two boards.

In the transmitting role, `RATE 10` sets the rate from **1 to 500 Hz** and `SIZE 32` sets the payload from **8 to 250 bytes**, the ESP-NOW limit. These ranges are narrower than the csi_tx values.

In the receiving role, `DISP STAT` or `DISP CSV` selects the display, `SC 44` selects the subcarrier shown in the summary from 0 to 63, `FORMAT RAW` selects RAW, AMP, or PHASE, and `LLTF 1` limits the array to 128 values. `FILTER AUTO` locks onto the first transmitter that is heard, `FILTER OFF` accepts everything, and a MAC address can also be given directly.

The `CSILLTF` through `CSISHIFT` commands, `TIME`, and the management commands work as they do on the receiver.

---

## 1.6 Free to Modify and Extend

This project is released under MIT License with complete freedom to modify, extend, and adapt. In this era of advancing AI, you are encouraged to freely modify this code for your own purposes.

---

## 1.7 License

MIT License (c) 2026 Hiroshi Matsuura

For details, refer to the LICENSE file.

---

## 1.8 Citation

If you use this project, a citation in the following form would be appreciated.

Matsuura, H. (2026). Arduino Wi-Fi CSI Tool. https://github.com/matsuura-hiroshi/Arduino-Wi-Fi_CSI_tool

---

## 1.9 Acknowledgment

This software is a result of research supported by the Ministry of Internal Affairs and Communications, Japan, under the Fundamental Technologies for Sustainable Efficient Radio Wave Use R&D Project (FORWARD).

---

## 1.10 Support

If you have problems or questions, please let us know on GitHub Issues.

Reply is not expected.

---

---

# 2. Arduino Wi-Fi CSI Tool（日本語）

本ソフトはArduino IDE 対応の Wi-Fi CSI センシングを目的として作りました．大学教育と簡単な実験に使うためのもので商用にするにはさらに改良する必要があるでしょう．

---

## 2.1 主な特徴

ボード起動後に，シリアルまたは UDP のコマンドで Wi-Fi チャネルや送信電力などの設定を動的に変更できます．設定は NVS に自動保存されるので，ボード再起動後も保持されます．

Arduino スケッチは 3 種類あります．受信機，送信機，そして 1 本で両方の役割になれる T-Beam 実証版です．これに加えて，波形描画用の Python スクリプト 2 本と，設定用のコマンドラインツールを同梱しています．

---

## 2.2 動作確認済み

Arduino IDE 版は T-Beam (LilyGO)，ESP32-WROOM，ESP32-S3 で動作確認済みです．

松浦，共同研究をしている企業，学生がそれぞれ独立して動作確認し，それぞれの研究用途で運用中です．

ESP-IDF 版も同梱していますが，実機での検証はまだ行われていません．

---

## 2.3 ファイル構成

同梱されているのは以下がすべてです．これ以外のファイルはありません．

```
Arduino-Wi-Fi_CSI_tool/
├── csi_ctl.py                        設定ツール（このフォルダで実行）
│
├── arduino/                          Arduino IDE 版
│   ├── csi_rx_arduino/
│   │   └── csi_rx_arduino.ino        受信機：softAP になり CSI を取得
│   ├── csi_tx_arduino/
│   │   └── csi_tx_arduino.ino        送信機：STA として接続しパケットを送る
│   ├── csi_tbeam/
│   │   └── csi_tbeam.ino             T-Beam 実証版：1 本で両方の役割
│   ├── serial_plot_csi_live.py       波形描画（このフォルダで実行）
│   └── serial_plot_csi_live_adjustable.py
│                                     波形描画・正規化つまみ付き
│
├── csi_rx/                           受信機の ESP-IDF 版（未検証）
│   └── main/main.c
└── csi_tx/                           送信機の ESP-IDF 版（未検証）
    └── main/main.c
```

`csi_ctl.py` はトップ階層に，波形描画スクリプトは `arduino/` の中にあります．実行場所が違う点にご注意ください．

---

## 2.4 環境構築

### Arduino IDE 版

Arduino IDE ＋ ESP32 ボードパッケージ 3.x 以降で動作します．ボードマネージャで esp32 by Espressif Systems のバージョン 3.x をインストールし，`arduino/csi_rx_arduino/csi_rx_arduino.ino` または `csi_tx_arduino.ino` を開きます．ボードを ESP32 Dev Module に選択して書き込みます．シリアルモニタは 921600 baud，改行 LF（または CR+LF）に設定してください．

受信機は既定で `myssid` というアクセスポイントをチャネル 6 に立て，送信機はその SSID に接続しにいきます．どちらも SSID・PASS コマンドで変更できます．

### T-Beam 実証版

T-Beam 2 台用の単一スケッチで，`arduino/csi_tbeam` に入っています．ボードは "T-Beam" または "ESP32 Dev Module" を選択します．

シリアルで `ROLE t` または `ROLE r` を打つと，役割を NVS に保存して再起動し，その役割で立ち上がります．`t` は ESP-NOW ブロードキャストを MCS0 固定で送信し，`r` は同一チャネルで CSI を受信します．2 台のチャネルが違うと一切受信できないのでご注意ください．

受信側の表示は `DISP STAT` と `DISP CSV` で切り替えます．STAT は毎秒 1 行の要約（rx/s・ロス率・RSSI・指定サブキャリアの振幅），CSV は 26 カラムの全データです．

チャネルは JP 設定で 1-13 が使えます．14 は 11b 専用のため CSI が取れません．

### ESP-IDF 版

コードは理論的には ESP-IDF 5.x で動くはずですが，実機での検証はまだ行われていません．

---

## 2.5 コマンド一覧

### 受信機（csi_rx_arduino / csi_rx）

Wi-Fi 設定として `SSID myssid`，`PASS mypassword` で AP の認証情報を設定します．パスワードが 8 文字未満だと WPA2 にできないため，認証なしのオープン AP になります．`CHANNEL 6` でチャネル 1-13，`BW 20` で帯域 20 または 40MHz，`TXPOWER 20` で電力 2-20dBm を設定します．`PROTOCOL BGN` で 11B・BG・BGN を選び，`MAXCONN 16` で最大接続数 1-16，`HIDDEN 0` で SSID 隠蔽を制御します．

CSI 取得設定として `CSILLTF 1`，`CSIHT 1`，`CSISTBC 1` で L-LTF・HT-LTF・STBC HT-LTF2 の有効化を制御します．`CSIMERGE 1` で LTF 統合を，`CSICHFILT 0` でチャネルフィルタを制御します．`CSISCALE 0`，`CSISHIFT 0` で手動スケーリングとシフト量 0-15 を設定します．

出力設定として `FORMAT RAW` で RAW（生 I/Q）・AMP（振幅）・PHASE（位相）を選びます．`MODE FULL` で FULL（26 カラム）または COMPACT（5 カラム）を選択します．`OUTPUT UDP` で UDP・SERIAL・BOTH を選び，`TARGET 192.168.4.255` で送出先（255 終わりでブロードキャスト），`PORT 5005` で送出ポートを設定します．`LLTF 1` で配列を先頭 128 値に制限し，`FILTER 2C:BC:BB:83:1E:D8` で指定 MAC のみ出力します（解除は `FILTER OFF`）．

`TIME` で unix 秒の実時刻を設定できます．`python csi_ctl.py time` で PC の時計を送れます．管理コマンドは `SHOW`，`SAVE`，`RESET`，`REBOOT`，`HELP` です．

### 送信機（csi_tx_arduino / csi_tx）

`SSID myssid`，`PASS mypassword` で接続先 AP を設定し，即座に再接続します．`TXPOWER 20` で電力 2-20dBm，`PROTOCOL BGN` で 11B・BG・BGN を選びます．

`RATE 100` でレート 1-1000Hz，`SIZE 8` でペイロード 1-1400B を設定します．`DSTIP 192.168.4.1`，`DSTPORT 5010` で送信先を設定しますが，通常は変更不要です．

管理コマンドは `SHOW`，`SAVE`，`RESET`，`REBOOT`，`HELP` です．

### T-Beam 実証版（csi_tbeam）

このスケッチは独自のコマンド体系を持ち，いくつかの範囲が上の送信機と異なります．

`ROLE t` / `ROLE r` で役割を設定し，保存して再起動します．`CHANNEL 6` でチャネル 1-13，`TXPOWER 20` で電力 2-20dBm を設定します．この 2 つは 2 台で一致させてください．

送信側では `RATE 10` でレート **1-500Hz**，`SIZE 32` でペイロード **8-250B** を設定します．後者は ESP-NOW の上限です．いずれも csi_tx より範囲が狭い点にご注意ください．

受信側では `DISP STAT` / `DISP CSV` で表示を切り替え，`SC 44` で要約に出すサブキャリア 0-63 を選び，`FORMAT RAW` で RAW・AMP・PHASE を選び，`LLTF 1` で配列を 128 値に制限します．`FILTER AUTO` は最初に受信した送信機に自動でロックし，`FILTER OFF` はすべて拾います．MAC を直接指定することもできます．

`CSILLTF` から `CSISHIFT` までの各コマンド，`TIME`，管理コマンドは受信機と同じように使えます．

---

## 2.6 自由な改造・拡張

このプロジェクトは MIT ライセンスの下で，改造・拡張・応用の完全な自由を提供します．AI が発達した現代において，このコードを自由に改造・拡張してください．

---

## 2.7 ライセンス

MIT License (c) 2026 Hiroshi Matsuura

詳細は LICENSE ファイルを参照してください．

---

## 2.8 引用

本プロジェクトを使用される場合，以下のように引用していただけると嬉しいです．

Matsuura, H. (2026). Arduino Wi-Fi CSI Tool. https://github.com/matsuura-hiroshi/Arduino-Wi-Fi_CSI_tool

---

## 2.9 謝辞

本ソフトウェアは，総務省「持続可能な電波有効利用のための基盤技術研究開発事業（FORWARD）」の支援を受けて実施した研究の成果の一部です．

---

## 2.10 サポート

問題や質問がある場合は，GitHub Issues でお知らせください．

返信は期待しないでください．
