# STM32F401CC ELRS 雙向 bench 測試

獨立的 CRSF 雙向鏈路驗證專案。只適用於已確認有 WeAct HID Bootloader、
APP 位址 `0x08004000` 的 STM32F401CCU6 板子，同時接 ES900TX 高頻頭與
ES900RX 接收機。協議層 (`src/crsf.c`/`crsf.h`) 移植自參考專案
`elrs_f401ccu6_platformio`，其餘（時脈、GPIO、USART、中斷）改成
暫存器級實作，不依賴 HAL，Vector Table 固定在 `0x08004000` 以跟
bootloader 共存（做法同 `tools/stm32_hid_led_test`）。

## 目的

驗證：

1. STM32 -> ES900TX -> ELRS 無線 -> ES900RX -> STM32：RC Channel 下行。
2. STM32 -> ES900RX -> ELRS 無線回傳 -> ES900TX -> STM32：Telemetry 回傳。

## 接線

| STM32F401CCU6 | 對象 | 說明 |
|---|---|---|
| PA9 / USART1 | ES900TX JR Pin 5 Signal/Data | 半雙工單線，建議串 1kΩ |
| GND | ES900TX JR Pin 4 GND | 共地 |
| （外部電源）| ES900TX JR Pin 3 VBAT/VCC | 不要吃 STM32 3.3V |
| PA2 / USART2_TX | ES900RX RX | 送 telemetry 給接收機 |
| PA3 / USART2_RX | ES900RX TX | 收接收機輸出的 CRSF |
| 5V / GND | ES900RX | 依接收機規格供電，共地 |

USART1：半雙工單線、400000 baud、8N1。
USART2：全雙工、420000 baud、8N1。
兩者都在 HSI 16MHz 下直接產生（不需要 PLL），400000 baud 為精確值，
420000 baud 實際約 421053 Hz（+0.25% 誤差）。

## 行為

- 開機後立即開始每 10 ms 送一包 CRSF RC Channels Packed frame 給 ES900TX。
- 每 2 秒自動切換 CH1：1000 → 1500 → 2000（其餘通道固定安全值，
  Fire/Arm/Emergency 這幾個通道恆為關閉）。
- 持續從 ES900RX 讀回 RC frame，比對前 8 個通道是否與送出值一致
  （容許 ±20us）。
- 每 1 秒送一包假電池 telemetry（CRSF Battery Sensor）給 ES900RX，
  若 ES900TX 端的 JR Signal/Data 線後續回傳同一種 frame，代表
  telemetry 回傳鏈路成功。

## PC13 LED 狀態燈

| 狀態 | 燈號 |
|---|---|
| 還沒確認「傳過去」（下行） | 慢閃，約 1Hz |
| 「傳過去」確認成功，「傳回來」（回傳）還沒確認 | 快閃，約 5Hz |
| 「傳過去」與「傳回來」都確認成功 | 恆亮 |

判讀基準：下行狀態超過 500ms 沒收到新的匹配 frame 視為過期；
回傳狀態超過 1500ms 沒收到新的 Battery frame 視為過期。
這顆燈是唯一的視覺化輸出（未使用第三顆 UART、USB-TTL 轉接器或網頁）。

## 記憶體與啟動

- 沿用 `tools/stm32_hid_led_test` 的記憶體配置：Bootloader 佔
  `0x08000000..0x08003FFF`；APP 起點 `0x08004000`，最大 240 KiB。
- 啟動碼禁止中斷、初始化資料/BSS；`prepare_hardware()` 清除繼承的
  NVIC/SysTick/USB/GPIO 狀態、切回 HSI 16MHz，只在設定完全部周邊
  （USART1/2、NVIC）之後才最後執行 `cpsie i` 解除全域中斷遮罩。
- Vector Table 位置經過核對 STMicroelectronics 官方
  `cmsis_device_f4/stm32f401xc.h` 的 `IRQn_Type`：
  `USART1_IRQn = 37`、`USART2_IRQn = 38`，對應到向量表 word index
  53、54；`build.py` 會在建置時自動檢查這兩個位置確實指向真正的
  handler，不是掉回 `Default_Handler`。

## 建置（不會燒錄）

需要 Python 3.9+、原生 `gcc`／`cc`（跑 host 端 CRSF 單元測試用）、
ARM GNU GCC（跑交叉編譯用，已驗證 PlatformIO
`toolchain-gccarmnoneeabi@1.120301.0`，GCC 12.3.1）。

```sh
python3 build.py
```

流程：先用原生編譯器編出 `test/host_test_crsf.c` 並執行，驗證
CRSF 打包/解析/CRC/串流同步邏輯（不牽涉任何硬體或交叉編譯工具鏈）；
通過才會繼續交叉編譯成 STM32 映像，並檢查向量位址、Thumb 指標、
SysTick/USART1/USART2 handler 是否正確掛上、映像大小、MSP、
未解析符號，產出 ELF、BIN、map、反組譯、section 表與
`verification.json`。

## 燒錄（尚未執行，需要另外授權）

用跟 `tools/stm32_hid_led_test` 相同、已修好 macOS 相容性 bug 的
`WeAct_HID_Flash-CLI` 原生工具：

```sh
<fixed-tool-path> read                                          # 先確認能讀到 MCU Info
<fixed-tool-path> dist/stm32f401cc_elrs_bidir_test.bin reboot    # 燒錄並重開機
```

燒錄前確認：

- STM32 只接 USB 供電，ES900TX 走獨立外部電源，不吃 STM32 3.3V。
- ES900TX 發射功率調到最低；TX 與 RX 天線間隔 30cm–1m，
  不要近距離長時間高功率測試。
- 目前沒有已知的舊 APP 備份；Download 一定會覆寫 APP 區。
- 只做一次 Download 嘗試；若卡住，停下回報而不是重試。

## 已驗證與待驗證

`python3 build.py` 已跑過：host 端 CRSF 單元測試（打包/解拆
round-trip、frame CRC 建構與驗證、串流解析器對亂碼前綴的重新同步、
壞 CRC 的拒收）全數通過；交叉編譯映像的靜態位址/向量/大小檢查全數
通過。尚未燒錄、尚未在實體 ES900TX/ES900RX 上觀察 PC13 狀態燈行為。

來源：
- 協議層移植自使用者提供的參考專案 `elrs_f401ccu6_platformio`
  （`src/crsf.c`、`README.md` 描述的接線/行為）。
- https://www.st.com/resource/en/reference_manual/rm0368-stm32f401xbc-and-stm32f401xde-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
- https://github.com/STMicroelectronics/cmsis_device_f4（`IRQn_Type` 核對來源）
