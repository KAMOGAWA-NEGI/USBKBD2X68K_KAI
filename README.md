# USBKBD2X68K_KAI


USBキーボードとUSBマウス／トラックボールを、Raspberry Pi Pico / RP2040系ボード経由で **SHARP X68000 のキーボードポート**へ接続するための変換アダプター用ファームウェアです。

X68000本体側からは、従来のキーボードポート接続キーボードおよびマウスとして扱われることを目標にしています。

## 概要

このプロジェクトは、RP2040のUSBホスト機能を利用してUSB HIDキーボード／マウスを読み取り、X68000のキーボードポート用シリアル信号へ変換します。

主な機能は以下の通りです。

- USBキーボード入力をX68000キーコードへ変換
- USBマウス／トラックボール入力をX68000マウスデータへ変換
- 複合USBレシーバーのHIDレポートに対応
- キーリピート処理
- X68000側KEYTxDコマンドによるマウスデータ送信
- 任意でMSCTRL入力によるマウス送信トリガーにも対応
- RP2040-Zero系のWS2812 LED、およびPico系の通常オンボードLEDによる状態表示
- `keymap.h` によるキーマップ変更

## 原作者クレジット

本プロジェクトは、以下の先行コード・知見を参考にRP2040向けへ移植・調整したものです。

- USBKBD2X68K 原作者: **たねけん氏**  
  `@taneken2000 / taneken`
- 及び: **zato氏**  
  `@z_alpha2 / ztto`

上記原作者の成果に敬意を表します。

## 対象ハードウェア

想定ボード:

- Raspberry Pi Pico
- Raspberry Pi Pico互換RP2040ボード
- Waveshare RP2040-Zero
- その他、Arduino IDE + Earle Philhower arduino-pico coreでUSBホスト動作できるRP2040系ボード

想定USB機器:

- USBキーボード
- キーボード＋マウス複合レシーバー

例:

- Logitech系キーボード／マウス共有ドングル
- Kensington SlimBladeなどのUSBトラックボール

すべてのUSB HID機器での動作を保証するものではありません。

## 開発環境

- Arduino IDE
- Earle Philhower arduino-pico core
- Adafruit TinyUSB Library
- 必要に応じて Adafruit NeoPixel Library

Arduino IDEの設定例:

```text
Board: 使用するRP2040ボード
USB Stack: Adafruit TinyUSB (Native)
CPU Speed: 標準設定
```

重要:

```text
Tools -> USB Stack -> Adafruit TinyUSB (Native)
```

を選択してください。  
NativeではないAdafruit TinyUSB設定では、USB機器を認識しても正常動作しない場合があります。

## ピン配置

標準設定では以下のGPIOを使用します。

| RP2040 GPIO | 方向 | X68000側信号 | 内容 |
|---:|:---:|---|---|
| GP0 | 出力 | KEYRxD | X68000へ送るキーボードデータ / UART0 TX / 2400bps 8N1 |
| GP1 | 入力 | KEYTxD | X68000からのコマンドデータ / UART0 RX / 2400bps 8N1 |
| GP2 | 入力 | MSCTRL | 任意接続。MSCTRLトリガーモード用 |
| GP8 | 出力 | MSDATA | X68000へ送るマウスデータ / UART1 TX / 4800bps 8N2 |
| GP16 | 出力 | WS2812 | RP2040-Zero系オンボードLED |
| GPIO25 / LED_BUILTIN | 出力 | LED | Pico系オンボードLED予備出力 |

### レベル変換について

RP2040のGPIOは3.3V系です。  
X68000側からRP2040へ入る信号、特に以下は **5Vから3.3Vへのレベル変換が必要**です。

- KEYTxD
- MSCTRL

X68000側へ出力する信号についても、実機や回路条件に応じてバッファやレベル変換回路を検討してください。

レベルシフターの一例として：https://akizukidenshi.com/catalog/g/g113837/

## USB接続

RP2040側をUSBホストとして動作させるため、USBキーボード／マウスはRP2040ボードのUSB端子へOTG経由で接続します。

ボードによっては、USBホスト動作用の5V供給やOTG配線が必要です。

## マウス送信方式

標準設定では、X68000側のKEYTxDコマンドを見てマウスデータを返す方式を使用します。

```cpp
static constexpr uint8_t MOUSE_TRIGGER_MODE = MOUSE_TRIGGER_KEYTXD_CMD;
```

設定値:

| 値 | 意味 |
|---|---|
| `MOUSE_TRIGGER_KEYTXD_CMD` | KEYTxDの`0x40`要求に応じてMSDATAを送信 |
| `MOUSE_TRIGGER_MSCTRL_EDGE` | MSCTRL立ち下がりでMSDATAを送信 |
| `MOUSE_TRIGGER_HYBRID` | KEYTxD方式とMSCTRL方式の両対応 |

マウスパケットが二重に送信される場合は、`MOUSE_TRIGGER_KEYTXD_CMD` を使用してください。

## マウス移動量調整

トラックボールや高DPIマウスでは移動量が大きすぎる場合があります。  
以下の値で調整できます。

```cpp
static constexpr bool INVERT_X = false;
static constexpr bool INVERT_Y = false;
static constexpr bool SWAP_XY  = false;
static constexpr int32_t MOTION_SCALE_NUM = 1;
static constexpr int32_t MOTION_SCALE_DEN = 4;
```

例:

- 移動が速すぎる場合: `MOTION_SCALE_DEN` を大きくする
- X/Yが逆の場合: `SWAP_XY` を `true`
- 向きが逆の場合: `INVERT_X` / `INVERT_Y` を `true`

## キーマップ

キーマップは `keymap.h` で定義します。  
USB HID Usage IDからX68000キーコードへ変換するテーブルです。

例:

```cpp
const uint8_t keytable[] PROGMEM = {
    // HID Usage ID -> X68000 key code
};
```

キーマップを変更したい場合は、`keymap.h` を編集して再ビルドしてください。


## ビルド手順

1. Arduino IDEをインストール
2. Earle Philhower arduino-pico coreを追加
3. Adafruit TinyUSB Libraryをインストール
4. WS2812搭載ボードの場合はAdafruit NeoPixel Libraryもインストール
5. `X68K_USBKM_RP2040_REV.A.ino` を開く
6. ボードを選択
7. `Tools -> USB Stack -> Adafruit TinyUSB (Native)` を選択
8. コンパイルしてRP2040ボードへ書き込み

## 使用方法

1. RP2040ボードとX68000キーボードポートを接続
2. USBキーボード／マウスをRP2040側USB端子へ接続
3. X68000本体を起動
4. LED状態を確認
5. キーボード入力とマウス移動を確認

## 注意事項

- X68000本体に接続するため、配線ミスやレベル変換不足に注意してください。
- RP2040のGPIOは5Vトレラントではありません。
- USBハブや複合レシーバーは機器によりHIDレポート形式が異なります。
- すべてのUSBキーボード／マウスで動作するわけではありません。
- 実機接続前に、可能であればロジックアナライザ等でKEYRxD / MSDATAの信号を確認してください。

## トラブルシュート

### USB機器を認識しない

- Arduino IDEで `Adafruit TinyUSB (Native)` が選ばれているか確認
- USB OTG配線を確認
- USB機器側への5V供給を確認
- USBハブを使わず直接接続して確認

### SHIFTを押すとマウスが動く

古い版では、SHIFT修飾ビットがマウスReport IDとして誤認される場合がありました。  
REV.Aではこの問題を修正しています。

### マウスが動かない

- `MOUSE_TRIGGER_MODE` を確認
- 標準では `MOUSE_TRIGGER_KEYTXD_CMD`
- MSCTRLを使う場合は配線とレベル変換を確認
- `MOUSE_REPORT_ID_HINT1/2/3` の値を変更して試す

### マウス移動が速すぎる／遅すぎる

`MOTION_SCALE_NUM` と `MOTION_SCALE_DEN` を調整してください。

