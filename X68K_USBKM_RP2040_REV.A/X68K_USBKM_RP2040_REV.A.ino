/*
  X68K_USBKM_RP2040_REV.A.ino

  USBキーボード + USBマウス/トラックボール -> X68000 キーボードポート変換アダプター

  原作者クレジット:
    - USBKBD2X68K 原作者: たねけん氏 (@taneken2000 / taneken)
    - X68000 USBキーボード/マウス関連コード原作者: zato氏 (@z_alpha2 / ztto)

  このRP2040版について:
    - 上記の元コードを参考に、RP2040ボード + Arduino IDE + TinyUSB Native環境向けに移植・調整した版。
    - REV.Aでは、SHIFTキー入力がマウスレポートとして誤認される問題と、オンボードLED表示を修正。

  対象環境:
    - Waveshare RP2040-Zero / RP2040系ボード
    - Arduino IDE
    - Earle Philhower arduino-pico core
    - Tools -> USB Stack -> "Adafruit TinyUSB (Native)"

  重要:
    Nativeではない "Adafruit TinyUSB" スタックでは、認識してもこの構成では正常動作しない場合がある。
    "Adafruit TinyUSB (Native)" を使用すること。

  X68000 キーボードポート配線:
    GP0  -> X68000 KEYRxD  : X68000へ送るキーボードデータ, 2400bps 8N1
    GP1  <- X68000 KEYTxD  : X68000からのコマンドデータ, 2400bps 8N1, 5V->3.3Vレベル変換必須
    GP2  <- X68000 MSCTRL  : 任意接続。MSCTRLマウストリガーモードでのみ使用, 5V->3.3Vレベル変換必須
    GP8  -> X68000 MSDATA  : X68000へ送るマウスデータ, 4800bps 8N2
    GP16 -> オンボードWS2812 : RP2040-Zero系ボードのデバッグLED
    LED_BUILTIN/GPIO25 -> Pico系ボードの通常オンボードLED予備出力

  標準のマウストリガーモード:
    標準ではKEYTxDコマンド方式を使用する。
    USBKBD2X68K系のキーボードポート接続で、X68000側からの0x40要求に応じてMSDATAを返す。

  キーリピート:
    USBキーを押し続けた場合は、元のUSBKBD2X68Kと同様にソフトウェア側でリピート送信する。
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "hardware/sync.h"
#include "keymap.h"

#if __has_include(<Adafruit_NeoPixel.h>)
  #include <Adafruit_NeoPixel.h>
  #define X68_HAVE_NEOPIXEL 1
#else
  #define X68_HAVE_NEOPIXEL 0
#endif

// -----------------------------------------------------------------------------
// ユーザー設定
// -----------------------------------------------------------------------------

static constexpr uint PIN_KEY_RXD = 0;   // RP2040 -> X68000 KEYRxD, UART0 TX
static constexpr uint PIN_KEY_TXD = 1;   // X68000 -> RP2040 KEYTxD, UART0 RX
static constexpr uint PIN_MSCTRL  = 2;   // 任意接続のX68000 MSCTRL入力
static constexpr uint PIN_MSDATA  = 8;   // RP2040 -> X68000 MSDATA, UART1 TX
#if defined(PIN_NEOPIXEL)
static constexpr uint PIN_WS2812  = PIN_NEOPIXEL;  // ボード定義にPIN_NEOPIXELがある場合はオンボードWS2812を使用
#else
static constexpr uint PIN_WS2812  = 16;            // Waveshare RP2040-Zeroの標準値
#endif

#if defined(LED_BUILTIN)
static constexpr int PIN_GPIO_LED = LED_BUILTIN;   // Raspberry Pi Pico/Pico互換ボードの通常LED
#else
static constexpr int PIN_GPIO_LED = 25;            // RP2040 Pico系ボード向けの一般的な予備値
#endif

static constexpr uint32_t KBD_BAUD = 2400;
static constexpr uint32_t MOUSE_BAUD = 4800;

// マウストリガーモード
static constexpr uint8_t MOUSE_TRIGGER_MSCTRL_EDGE = 0;
static constexpr uint8_t MOUSE_TRIGGER_KEYTXD_CMD  = 1;
static constexpr uint8_t MOUSE_TRIGGER_HYBRID      = 2;

// 標準設定: 2系統のトリガー方式を受け付ける場合の説明
// - KEYTxDコマンド方式は元のUSBKBD2X68K.inoの挙動に合わせる。
// - MSCTRLエッジ方式は動作確認済みのRP2040マウス専用スケッチに合わせる。
// マウスパケットが二重に出る場合はMOUSE_TRIGGER_KEYTXD_CMDに設定する。
static constexpr uint8_t MOUSE_TRIGGER_MODE = MOUSE_TRIGGER_KEYTXD_CMD;

// KEYTXD_CMD/HYBRIDモード時:
// false = 元のUSBKBD2X68K相当。0x41 -> 0x40の遷移時だけ送信する。
// true  = 0x40を受信するたび送信する。0x41なしで0x40が繰り返される実機向け。
static constexpr bool MOUSE_SEND_ON_EVERY_0X40 = true;

// MSCTRL立ち下がりからUART先頭スタートビットまでの遅延。MSCTRLモードでのみ使用。
static constexpr uint32_t TX_DELAY_US = 850;
static constexpr uint32_t TX_BUSY_US = 7600;
static constexpr int64_t TX_TIMER_PERIOD_US = -50;
static constexpr bool REQUIRE_MSCTRL_LOW_AT_TX = false;

// マウス移動量の調整。SlimBladeなどのトラックボールでは小さめにスケーリングすることが多い。
static constexpr bool INVERT_X = false;
static constexpr bool INVERT_Y = false;
static constexpr bool SWAP_XY  = false;
static constexpr int32_t MOTION_SCALE_NUM = 1;
static constexpr int32_t MOTION_SCALE_DEN = 4;
static constexpr int32_t ACCUM_LIMIT = 4096;

// キーリピート設定。元のUSBKBD2X68Kの挙動を基準にする。
static constexpr uint8_t REPEATTIME_TICKS = 5;        // 5 * 50ms = リピート開始まで約250ms
static constexpr uint8_t MAXKEYENTRY = 6;
static constexpr uint8_t EMPTY_USAGE = 0;
static uint32_t REP_INTERVAL_MS = 50;                 // 元コードの標準値

// M212 + K240共有ドングルのような複合レシーバではReportプロトコルの方が安全。
static constexpr bool FORCE_HID_BOOT_PROTOCOL = false;

// TinyUSBが同じドングルをキーボード扱いしても、Report ID付きマウスパケットを受け付ける。
static constexpr bool ACCEPT_MOUSE_ON_KEYBOARD_INTERFACE = true;
static constexpr bool ACCEPT_MOUSE_ON_GENERIC_INTERFACE  = true;

// 複合レシーバで使われやすいReport ID。マウスで緑点滅しない場合はまず3/4を試す。
static constexpr uint8_t MOUSE_REPORT_ID_HINT1 = 2;
static constexpr uint8_t MOUSE_REPORT_ID_HINT2 = 3;
static constexpr uint8_t MOUSE_REPORT_ID_HINT3 = 4;

// デバッグ用。任意のHIDレポート受信時にAキーを強制送信する。通常はfalseのまま。
static constexpr bool DIAG_SEND_A_ON_ANY_HID_REPORT = false;

// X68000マウスパケットへのボタン割り当て。
static constexpr uint8_t X68_BUTTON_LEFT  = 0x01;
static constexpr uint8_t X68_BUTTON_RIGHT = 0x02;

// -----------------------------------------------------------------------------
// USBホスト
// -----------------------------------------------------------------------------

Adafruit_USBH_Host USBHost;

static constexpr uint8_t MAX_USB_ADDR = 8;
static constexpr uint8_t MAX_HID_INST = 4;

enum HidKind : uint8_t {
  HID_KIND_NONE = 0,
  HID_KIND_KEYBOARD,
  HID_KIND_MOUSE,
  HID_KIND_GENERIC
};

static HidKind g_hidKind[MAX_USB_ADDR][MAX_HID_INST];
static volatile uint8_t g_keyboardMountedCount = 0;
static volatile uint8_t g_mouseMountedCount = 0;

// キーボード状態。Boot Keyboard互換形式。
static uint8_t g_prevMod = 0;
static uint8_t g_prevKeys[6] = {0, 0, 0, 0, 0, 0};

// ソフトウェアキーリピート用テーブル。X68000コードではなくHID Usage IDを保持する。
static uint8_t g_repeatUsage[MAXKEYENTRY] = {0, 0, 0, 0, 0, 0};
static uint8_t g_repeatWait[MAXKEYENTRY]  = {0, 0, 0, 0, 0, 0};
static uint32_t g_lastRepeatMs = 0;

// マウス共有状態。
static volatile int32_t g_accumX = 0;
static volatile int32_t g_accumY = 0;
static volatile uint8_t g_buttons = 0;
static volatile bool g_dataPending = false;

// MSCTRL/TXスケジューラ状態。マウス専用方式で使用。
static volatile bool g_ctrlPending = false;
static volatile uint32_t g_ctrlEdgeUs = 0;
static volatile uint32_t g_txBusyUntilUs = 0;
static volatile uint32_t g_lastMouseTxStartUs = 0;
static volatile uint32_t g_lastKbdReportUs = 0;
static volatile uint32_t g_lastMouseReportUs = 0;
static volatile uint32_t g_lastKeySendUs = 0;
static volatile uint32_t g_lastKeyTxdCmdUs = 0;
static repeating_timer_t g_txTimer;

static uint8_t g_oldKeyTxdCmd = 0x41;

#if X68_HAVE_NEOPIXEL
static Adafruit_NeoPixel g_pixel(1, PIN_WS2812, NEO_GRB + NEO_KHZ800);
#endif

// -----------------------------------------------------------------------------
// ユーティリティ
// -----------------------------------------------------------------------------

static inline int32_t clamp32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int8_t clampToI8(int32_t v) {
  if (v < -128) return -128;
  if (v > 127) return 127;
  return static_cast<int8_t>(v);
}

static inline bool timePassed(uint32_t now, uint32_t target) {
  return static_cast<int32_t>(now - target) >= 0;
}

static void setPixel(uint8_t r, uint8_t g, uint8_t b) {
#if X68_HAVE_NEOPIXEL
  g_pixel.setPixelColor(0, g_pixel.Color(r, g, b));
  g_pixel.show();
#endif

  // Raspberry Pi Pico/Pico互換ボードのような通常単色LED搭載ボード向けの予備処理。
  // WS2812データ線と同じピンは駆動しない。
  // 同じピンを通常LEDとして駆動するとNeoPixel信号が壊れるため。
  if (PIN_GPIO_LED >= 0 && static_cast<uint>(PIN_GPIO_LED) != PIN_WS2812) {
    digitalWrite(PIN_GPIO_LED, (r || g || b) ? HIGH : LOW);
  }
}

static void flashPixel(uint8_t r, uint8_t g, uint8_t b) {
  setPixel(r, g, b);
}

static void initStatusLED() {
  if (PIN_GPIO_LED >= 0 && static_cast<uint>(PIN_GPIO_LED) != PIN_WS2812) {
    pinMode(PIN_GPIO_LED, OUTPUT);
    digitalWrite(PIN_GPIO_LED, LOW);
  }

#if X68_HAVE_NEOPIXEL
  g_pixel.begin();
  g_pixel.setBrightness(24);
#endif
  setPixel(16, 0, 0);
}

static void updateStatusLED() {
  static uint32_t lastMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastMs < 120) return;
  lastMs = nowMs;

  uint32_t nowUs = time_us_32();
  if (static_cast<uint32_t>(nowUs - g_lastMouseTxStartUs) < 160000) {
    setPixel(24, 16, 0);      // 黄 = MSDATAパケット送信
  } else if (static_cast<uint32_t>(nowUs - g_lastMouseReportUs) < 160000) {
    setPixel(0, 24, 0);       // 緑 = マウスHIDレポート解析
  } else if (static_cast<uint32_t>(nowUs - g_lastKeyTxdCmdUs) < 120000) {
    setPixel(20, 20, 20);     // 白 = KEYTxDコマンド受信
  } else if (static_cast<uint32_t>(nowUs - g_lastKeySendUs) < 120000) {
    setPixel(24, 0, 24);      // 紫 = キーボードバイト送信
  } else if (static_cast<uint32_t>(nowUs - g_lastKbdReportUs) < 120000) {
    setPixel(0, 0, 24);       // 青 = キーボードHIDレポート解析
  } else if (g_keyboardMountedCount && g_mouseMountedCount) {
    setPixel(0, 14, 18);      // シアン系 = キーボード・マウス両方接続
  } else if (g_keyboardMountedCount) {
    setPixel(0, 0, 24);       // 青
  } else if (g_mouseMountedCount) {
    setPixel(0, 18, 0);       // 緑
  } else {
    if ((nowMs / 400) & 1) setPixel(16, 0, 0);
    else setPixel(0, 0, 0);
  }
}

// -----------------------------------------------------------------------------
// X68000 UART
// -----------------------------------------------------------------------------

static void setupX68Uarts() {
  // UART0: キーボード用の双方向リンク。
  // GP0 = X68000 KEYRxDへのTX、GP1 = X68000 KEYTxDからのRX。
  gpio_set_function(PIN_KEY_RXD, GPIO_FUNC_UART);
  gpio_set_function(PIN_KEY_TXD, GPIO_FUNC_UART);
  uart_init(uart0, KBD_BAUD);
  uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(uart0, true);

  // UART1: マウスデータ出力。RP2040ではGP8をUART1 TXとして使用。
  gpio_set_function(PIN_MSDATA, GPIO_FUNC_UART);
  uart_init(uart1, MOUSE_BAUD);
  uart_set_format(uart1, 8, 2, UART_PARITY_NONE);
  uart_set_fifo_enabled(uart1, true);
}

static void sendKbdByte(uint8_t code) {
  uart_putc_raw(uart0, code);
  g_lastKeySendUs = time_us_32();
  flashPixel(24, 0, 24);  // 紫 = KEYRxD送信
}

static void sendKeyPressRelease(uint8_t code) {
  sendKbdByte(code);
  delay(4);
  sendKbdByte(code | 0x80);
}

static uint8_t readX68CodeFromKeytable(uint8_t usage);

// -----------------------------------------------------------------------------
// キーリピート処理
// -----------------------------------------------------------------------------

static void clearRepeatTable() {
  for (uint8_t i = 0; i < MAXKEYENTRY; i++) {
    g_repeatUsage[i] = EMPTY_USAGE;
    g_repeatWait[i] = 0;
  }
}

static void addRepeatUsage(uint8_t usage) {
  if (usage == EMPTY_USAGE) return;

  for (uint8_t i = 0; i < MAXKEYENTRY; i++) {
    if (g_repeatUsage[i] == usage) {
      g_repeatWait[i] = REPEATTIME_TICKS;
      return;
    }
  }

  for (uint8_t i = 0; i < MAXKEYENTRY; i++) {
    if (g_repeatUsage[i] == EMPTY_USAGE) {
      g_repeatUsage[i] = usage;
      g_repeatWait[i] = REPEATTIME_TICKS;
      return;
    }
  }
}

static void delRepeatUsage(uint8_t usage) {
  for (uint8_t i = 0; i < MAXKEYENTRY; i++) {
    if (g_repeatUsage[i] == usage) {
      g_repeatUsage[i] = EMPTY_USAGE;
      g_repeatWait[i] = 0;
      return;
    }
  }
}

static void serviceKeyRepeat() {
  uint32_t nowMs = millis();
  if (nowMs - g_lastRepeatMs < REP_INTERVAL_MS) return;
  g_lastRepeatMs = nowMs;

  for (uint8_t i = 0; i < MAXKEYENTRY; i++) {
    uint8_t usage = g_repeatUsage[i];
    if (usage == EMPTY_USAGE) continue;

    if (g_repeatWait[i] != 0) {
      g_repeatWait[i]--;
      continue;
    }

    uint8_t code = readX68CodeFromKeytable(usage);
    if (code != 0x00) {
      // USBKBD2X68Kと同様、リピート時はmake+breakではなくmakeコードだけを再送する。
      sendKbdByte(code);
    }
  }
}

// -----------------------------------------------------------------------------
// キーボードHID処理
// -----------------------------------------------------------------------------

static uint8_t readX68CodeFromKeytable(uint8_t usage) {
  if (usage >= sizeof(keytable)) return 0x00;
  return pgm_read_byte(&keytable[usage]);
}

static bool containsUsage(uint8_t const keys[6], uint8_t usage) {
  for (uint8_t i = 0; i < 6; i++) {
    if (keys[i] == usage) return true;
  }
  return false;
}

static void sendModTransition(uint8_t before, uint8_t after, uint8_t mask, uint8_t x68Code) {
  bool was = (before & mask) != 0;
  bool now = (after  & mask) != 0;
  if (was == now) return;
  sendKbdByte(now ? x68Code : static_cast<uint8_t>(x68Code | 0x80));
}

static void handleModifierChanges(uint8_t before, uint8_t after) {
  // 左右ShiftはどちらもX68000のSHIFTへ割り当てる。片方を押し続けている間に
  // もう片方を離してもSHIFTが解除されないよう、左右をまとめて扱う。
  bool beforeShift = (before & (0x02 | 0x20)) != 0;
  bool afterShift  = (after  & (0x02 | 0x20)) != 0;
  if (beforeShift != afterShift) sendKbdByte(afterShift ? 0x70 : 0xF0);

  sendModTransition(before, after, 0x01, 0x71); // 左Ctrl
  sendModTransition(before, after, 0x04, 0x55); // 左Alt -> XF1
  sendModTransition(before, after, 0x08, 0x5F); // 左GUI -> ひらがな
  sendModTransition(before, after, 0x10, 0x73); // 右Ctrl -> OPT.2
  sendModTransition(before, after, 0x40, 0x59); // 右Alt -> XF5
  sendModTransition(before, after, 0x80, 0x72); // 右GUI -> OPT.1
}

static uint8_t scoreKeyboardReport(uint8_t const* report, uint16_t len, uint8_t offset) {
  if (len < offset + 8) return 0;
  uint8_t score = 0;
  for (uint8_t i = 2; i < 8; i++) {
    uint8_t u = report[offset + i];
    if (u == 0) continue;
    // 通常のBoot Keyboard Usage範囲。このkeymapは0x8Bまで定義している。
    if (u >= 0x04 && u < sizeof(keytable)) score++;
  }
  return score;
}

static bool parseBootKeyboardReport(uint8_t const* report, uint16_t len,
                                    uint8_t* mod, uint8_t keys[6]) {
  if (len < 8) return false;

  uint8_t offset = 0;
  if (len >= 9) {
    uint8_t s0 = scoreKeyboardReport(report, len, 0);
    uint8_t s1 = scoreKeyboardReport(report, len, 1);
    if (s1 > s0) offset = 1; // 先頭にReport IDが付加されている
  }

  if (len < offset + 8) return false;
  *mod = report[offset + 0];
  for (uint8_t i = 0; i < 6; i++) keys[i] = report[offset + 2 + i];
  return true;
}

static bool isBootKeyboardLikeReport(uint8_t const* report, uint16_t len) {
  uint8_t mod = 0;
  uint8_t keys[6] = {0, 0, 0, 0, 0, 0};
  if (!parseBootKeyboardReport(report, len, &mod, keys)) return false;

  uint8_t offset = 0;
  if (len >= 9) {
    uint8_t s0 = scoreKeyboardReport(report, len, 0);
    uint8_t s1 = scoreKeyboardReport(report, len, 1);
    if (s1 > s0) offset = 1;
  }

  // Boot Keyboardレポートの1バイト目は予約領域で、0でなければならない。
  // これにより、SHIFT=0x02などの修飾キーバイトを
  // マウスReport ID 2として誤読することを防ぐ。
  if (report[offset + 1] != 0x00) return false;

  for (uint8_t i = 0; i < 6; i++) {
    uint8_t u = keys[i];
    if (u == 0) continue;
    // 0x01..0x03はHIDキーボードのエラー状態。キーボード系として受け付け、
    // 偽のマウス移動として流さない。キー送信は行わない。
    if (u <= 0x03) continue;
    if (u >= sizeof(keytable)) return false;
  }
  (void)mod;
  return true;
}

static bool handleKeyboardReport(uint8_t const* report, uint16_t len) {
  uint8_t mod = 0;
  uint8_t keys[6] = {0, 0, 0, 0, 0, 0};
  if (!parseBootKeyboardReport(report, len, &mod, keys)) return false;

  g_lastKbdReportUs = time_us_32();
  flashPixel(0, 0, 24); // 青 = キーボードレポート

  handleModifierChanges(g_prevMod, mod);

  // キー押下: 現在あり、前回なし。
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t usage = keys[i];
    if (usage == 0) continue;
    if (!containsUsage(g_prevKeys, usage)) {
      uint8_t code = readX68CodeFromKeytable(usage);
      if (code != 0x00) {
        sendKbdByte(code);
        addRepeatUsage(usage);
      }
    }
  }

  // キー解放: 前回あり、現在なし。
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t usage = g_prevKeys[i];
    if (usage == 0) continue;
    if (!containsUsage(keys, usage)) {
      uint8_t code = readX68CodeFromKeytable(usage);
      if (code != 0x00) {
        sendKbdByte(code | 0x80);
        delRepeatUsage(usage);
      }
    }
  }

  g_prevMod = mod;
  memcpy(g_prevKeys, keys, sizeof(g_prevKeys));
  return true;
}

// -----------------------------------------------------------------------------
// マウスHID処理
// -----------------------------------------------------------------------------

static void accumulateMouseDelta(int32_t dx, int32_t dy, uint8_t buttons) {
  if (SWAP_XY) {
    int32_t t = dx;
    dx = dy;
    dy = t;
  }

  uint32_t irq = save_and_disable_interrupts();
  g_accumX = clamp32(g_accumX + dx, -ACCUM_LIMIT, ACCUM_LIMIT);
  g_accumY = clamp32(g_accumY + dy, -ACCUM_LIMIT, ACCUM_LIMIT);
  g_buttons = buttons;
  if (dx != 0 || dy != 0 || (buttons & 0x03) != 0) g_dataPending = true;
  restore_interrupts(irq);
}

static bool mouseReportIdLooksAllowed(uint8_t id) {
  if (id == MOUSE_REPORT_ID_HINT1 || id == MOUSE_REPORT_ID_HINT2 || id == MOUSE_REPORT_ID_HINT3) return true;
  return (id >= 2 && id <= 8);
}

static bool parseBootOrSimpleMouseReport(uint8_t const* report, uint16_t len,
                                         uint8_t* buttons, int32_t* dx, int32_t* dy,
                                         HidKind kind) {
  if (len < 3) return false;

  // 真のBoot Mouseレポート: ボタン、X、Y、任意のホイール。
  // キーボードクラスのインターフェースには適用しない。Boot Keyboardレポートが
  // マウス移動として誤読される可能性があるため。
  if (len == 3 || len == 4) {
    if (kind == HID_KIND_MOUSE || (kind == HID_KIND_GENERIC && ACCEPT_MOUSE_ON_GENERIC_INTERFACE)) {
      *buttons = report[0] & 0x07;
      *dx = static_cast<int8_t>(report[1]);
      *dy = static_cast<int8_t>(report[2]);
      return true;
    }
  }

  // Report ID + Boot Mouse風パケット。この経路は
  // M212+K240共有レシーバのような複合デバイスで重要。
  if (len >= 4 && mouseReportIdLooksAllowed(report[0]) && (report[1] & 0xF8) == 0x00) {
    if (kind == HID_KIND_MOUSE || kind == HID_KIND_GENERIC ||
        (kind == HID_KIND_KEYBOARD && ACCEPT_MOUSE_ON_KEYBOARD_INTERFACE)) {
      *buttons = report[1] & 0x07;
      *dx = static_cast<int8_t>(report[2]);
      *dy = static_cast<int8_t>(report[3]);
      return true;
    }
  }

  // Report ID + ボタン + 16bit相対X/Y形式。
  if (len >= 6 && mouseReportIdLooksAllowed(report[0]) && (report[1] & 0xF8) == 0x00) {
    if (kind == HID_KIND_MOUSE || kind == HID_KIND_GENERIC ||
        (kind == HID_KIND_KEYBOARD && ACCEPT_MOUSE_ON_KEYBOARD_INTERFACE)) {
      int16_t x16 = static_cast<int16_t>(static_cast<uint16_t>(report[2]) | (static_cast<uint16_t>(report[3]) << 8));
      int16_t y16 = static_cast<int16_t>(static_cast<uint16_t>(report[4]) | (static_cast<uint16_t>(report[5]) << 8));
      *buttons = report[1] & 0x07;
      *dx = x16;
      *dy = y16;
      return true;
    }
  }

  // 動作確認済みマウス専用スケッチ由来の保守的なフォールバック。ただし
  // 実マウス/汎用インターフェースに限定する。
  if (kind == HID_KIND_MOUSE || (kind == HID_KIND_GENERIC && ACCEPT_MOUSE_ON_GENERIC_INTERFACE)) {
    *buttons = report[0] & 0x07;
    *dx = static_cast<int8_t>(report[1]);
    *dy = static_cast<int8_t>(report[2]);
    return true;
  }

  return false;
}

static bool handleMouseReport(uint8_t const* report, uint16_t len, HidKind kind) {
  uint8_t buttons = 0;
  int32_t dx = 0;
  int32_t dy = 0;
  if (!parseBootOrSimpleMouseReport(report, len, &buttons, &dx, &dy, kind)) return false;

  g_lastMouseReportUs = time_us_32();
  flashPixel(0, 24, 0); // 緑 = マウスレポート
  accumulateMouseDelta(dx, dy, buttons);
  return true;
}

static void sendX68MousePacketFromAccumulator(uint32_t nowUs, bool respectBusy) {
  if (respectBusy && !timePassed(nowUs, g_txBusyUntilUs)) return;

  uint32_t irq = save_and_disable_interrupts();
  int32_t rawX = g_accumX;
  int32_t rawY = g_accumY;
  uint8_t buttons = g_buttons;
  restore_interrupts(irq);

  int32_t sx = rawX;
  int32_t sy = rawY;
  if (MOTION_SCALE_DEN != 0) {
    sx = (sx * MOTION_SCALE_NUM) / MOTION_SCALE_DEN;
    sy = (sy * MOTION_SCALE_NUM) / MOTION_SCALE_DEN;
  }

  if (INVERT_X) sx = -sx;
  if (INVERT_Y) sy = -sy;

  int8_t outX = clampToI8(sx);
  int8_t outY = clampToI8(sy);

  uint8_t status = 0;
  if (buttons & 0x01) status |= X68_BUTTON_LEFT;
  if (buttons & 0x02) status |= X68_BUTTON_RIGHT;

  uart_putc_raw(uart1, status);
  uart_putc_raw(uart1, static_cast<uint8_t>(outX));
  uart_putc_raw(uart1, static_cast<uint8_t>(outY));

  // 全データを消さず、未送信の移動量を残す。これによりトラックボールの
  // 移動量欠落を抑えつつ、元の3バイトパケット形式を維持する。
  int32_t consumeX = outX;
  int32_t consumeY = outY;
  if (INVERT_X) consumeX = -consumeX;
  if (INVERT_Y) consumeY = -consumeY;
  if (MOTION_SCALE_NUM != 0) {
    consumeX = (consumeX * MOTION_SCALE_DEN) / MOTION_SCALE_NUM;
    consumeY = (consumeY * MOTION_SCALE_DEN) / MOTION_SCALE_NUM;
  }

  irq = save_and_disable_interrupts();
  g_accumX = clamp32(g_accumX - consumeX, -ACCUM_LIMIT, ACCUM_LIMIT);
  g_accumY = clamp32(g_accumY - consumeY, -ACCUM_LIMIT, ACCUM_LIMIT);
  g_dataPending = (g_accumX != 0 || g_accumY != 0 || (g_buttons & 0x03) != 0);
  restore_interrupts(irq);

  g_lastMouseTxStartUs = nowUs;
  g_txBusyUntilUs = nowUs + TX_BUSY_US;
  flashPixel(24, 16, 0); // 黄 = MSDATA送信
}

static bool txTimerCallback(repeating_timer_t*) {
  if (MOUSE_TRIGGER_MODE != MOUSE_TRIGGER_MSCTRL_EDGE && MOUSE_TRIGGER_MODE != MOUSE_TRIGGER_HYBRID) return true;

  uint32_t nowUs = time_us_32();
  if (g_ctrlPending && timePassed(nowUs, g_ctrlEdgeUs + TX_DELAY_US)) {
    g_ctrlPending = false;
    if (!REQUIRE_MSCTRL_LOW_AT_TX || gpio_get(PIN_MSCTRL) == 0) {
      sendX68MousePacketFromAccumulator(nowUs, true);
    }
  }
  return true;
}

static void msctrlISR() {
  uint32_t nowUs = time_us_32();
  if (gpio_get(PIN_MSCTRL) == 0) {
    g_ctrlEdgeUs = nowUs;
    g_ctrlPending = true;
  }
}

static void pollKeyTxdForMouseCommand() {
  if (MOUSE_TRIGGER_MODE != MOUSE_TRIGGER_KEYTXD_CMD && MOUSE_TRIGGER_MODE != MOUSE_TRIGGER_HYBRID) return;

  while (uart_is_readable(uart0)) {
    uint8_t c = uart_getc(uart0);
    if (c == 0x40 || c == 0x41) {
      g_lastKeyTxdCmdUs = time_us_32();
      flashPixel(20, 20, 20); // 白 = KEYTxDコマンド

      bool send = false;
      if (MOUSE_SEND_ON_EVERY_0X40) {
        send = (c == 0x40);
      } else {
        send = (c == 0x40 && g_oldKeyTxdCmd == 0x41);
      }

      if (send) {
        // 元のUSBKBD2X68Kでは、KEYTxDが0x41 -> 0x40に変化した時点で即送信する。
        // ここではMSCTRL用のbusy抑制を適用しない。
        sendX68MousePacketFromAccumulator(time_us_32(), false);
      }
      g_oldKeyTxdCmd = c;
    } else if ((c >> 4) == 0x07) {
      // 元のUSBKBD2X68Kのリピート間隔コマンド。
      uint8_t n = c & 0x0F;
      REP_INTERVAL_MS = 30 + static_cast<uint32_t>(n) * static_cast<uint32_t>(n) * 5;
    }

    // 必要なら他のKEYTxDコマンドバイト処理をここに追加する。
  }
}

// -----------------------------------------------------------------------------
// TinyUSBコールバック
// -----------------------------------------------------------------------------

extern "C" void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance,
                                  uint8_t const* desc_report, uint16_t desc_len) {
  (void)desc_report;
  (void)desc_len;

  HidKind kind = HID_KIND_GENERIC;
  uint8_t proto = tuh_hid_interface_protocol(dev_addr, instance);

  if (proto == HID_ITF_PROTOCOL_KEYBOARD) {
    kind = HID_KIND_KEYBOARD;
    g_keyboardMountedCount++;
    if (FORCE_HID_BOOT_PROTOCOL) tuh_hid_set_protocol(dev_addr, instance, HID_PROTOCOL_BOOT);
  } else if (proto == HID_ITF_PROTOCOL_MOUSE) {
    kind = HID_KIND_MOUSE;
    g_mouseMountedCount++;
    if (FORCE_HID_BOOT_PROTOCOL) tuh_hid_set_protocol(dev_addr, instance, HID_PROTOCOL_BOOT);
  }

  if (dev_addr < MAX_USB_ADDR && instance < MAX_HID_INST) {
    g_hidKind[dev_addr][instance] = kind;
  }

  tuh_hid_receive_report(dev_addr, instance);
}

extern "C" void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  if (dev_addr < MAX_USB_ADDR && instance < MAX_HID_INST) {
    HidKind kind = g_hidKind[dev_addr][instance];
    if (kind == HID_KIND_KEYBOARD && g_keyboardMountedCount) g_keyboardMountedCount--;
    if (kind == HID_KIND_MOUSE && g_mouseMountedCount) g_mouseMountedCount--;
    g_hidKind[dev_addr][instance] = HID_KIND_NONE;
  }
}

extern "C" void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance,
                                            uint8_t const* report, uint16_t len) {
  HidKind kind = HID_KIND_GENERIC;
  if (dev_addr < MAX_USB_ADDR && instance < MAX_HID_INST) {
    kind = g_hidKind[dev_addr][instance];
  }

  if (DIAG_SEND_A_ON_ANY_HID_REPORT) {
    sendKeyPressRelease(0x1E); // A
  }

  // REV.A修正:
  // キーボードクラスのレポートを先にマウスとして解析しない。Boot Keyboard
  // レポートでは先頭バイトが修飾キービットマップで、左Shiftは0x02。REV.0では
  // 0x02をマウスReport IDとして受け付けてしまい、SHIFT+キーが
  // キーボード入力ではなくマウス移動として消費されていた。現在はキーボード風レポートを
  // マウス用フォールバックより前に処理する。
  bool used = false;

  if (kind == HID_KIND_KEYBOARD) {
    if (isBootKeyboardLikeReport(report, len)) {
      used = handleKeyboardReport(report, len);
    }
    if (!used && ACCEPT_MOUSE_ON_KEYBOARD_INTERFACE) {
      used = handleMouseReport(report, len, kind);
    }
  } else if (kind == HID_KIND_MOUSE) {
    used = handleMouseReport(report, len, kind);
  } else {
    if (isBootKeyboardLikeReport(report, len)) {
      used = handleKeyboardReport(report, len);
    }
    if (!used) {
      used = handleMouseReport(report, len, kind);
    }
    if (!used && len >= 8) {
      used = handleKeyboardReport(report, len);
    }
  }

  tuh_hid_receive_report(dev_addr, instance);
}

// -----------------------------------------------------------------------------
// Arduino setup / loop
// -----------------------------------------------------------------------------

void setup() {
  initStatusLED();
  setPixel(12, 4, 0);

  setupX68Uarts();

  if (MOUSE_TRIGGER_MODE == MOUSE_TRIGGER_MSCTRL_EDGE || MOUSE_TRIGGER_MODE == MOUSE_TRIGGER_HYBRID) {
    pinMode(PIN_MSCTRL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_MSCTRL), msctrlISR, FALLING);
    add_repeating_timer_us(TX_TIMER_PERIOD_US, txTimerCallback, nullptr, &g_txTimer);
  } else {
    pinMode(PIN_MSCTRL, INPUT_PULLUP); // 純粋なKEYTxDモードでは未使用
  }

  clearRepeatTable();

  // RP2040内蔵USBコントローラをホストとして使う場合、root hub portは0。
  USBHost.begin(0);

  setPixel(0, 0, 12);
}

void loop() {
  USBHost.task();
  pollKeyTxdForMouseCommand();
  serviceKeyRepeat();
  updateStatusLED();
}
