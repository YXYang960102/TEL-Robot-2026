# Mega ↔ STM32 UART 雙向獨立測試

接 ELRS RX 之前，先證明 STM32 和 Mega 之間的 UART 兩個方向都真的通。
這是獨立的測試韌體：Mega 這邊只編這個測試，不含馬達或其他子系統；
STM32 這邊不碰 USART2（ES900RX）。

**目前狀態：只做過編譯 + 電腦端測試，沒有燒錄、沒有實機跑過。**

## 接線（照 Jeremy 2026-10-11 的說明，未再檢查電路）

```
Mega D16 (TX2) ──電平轉換── STM32 PA10 (USART1 RX)
Mega D17 (RX2) ──電平轉換── STM32 PA9  (USART1 TX)
```
115200 8N1 全雙工。Orin 用的 D18/D19（Serial1）不動。
STM32 是一塊全新、沒燒過的 WeAct STM32F401CCU6（Black Pill）。

## 協定（細節見 `src/Bench/UartLinkTest/ulink.h`）

每個封包是一行 ASCII：`<內容>*<CRC-8 兩位 hex>\n`。

| 類型 | 方向 | 用途 |
|---|---|---|
| `D,<from>,<run>,<seq>,<payload>` | 雙方 | 資料，每 100 ms 主動送一次 |
| `A,<from>,<run>,<seq>,<payload>` | 雙方 | 收到 D 回的 ACK，原樣回傳流水號和內容 |
| `C,<run>,<1/0>` | Mega→STM32 | 開始/結束一輪測試 |
| `S,<run>,...` | STM32→Mega | STM32 自己的計數，每 250 ms 一次 |

- payload 依（來源, run, seq）算出 8 位 hex，接收端會驗證內容，不只驗證格式。
- ACK 不會觸發 ACK。一次只等一個 ACK，50 ms 沒回就記為逾時。
  晚到、重複、內容不符的 ACK 各自計數，都不算成功。
- 接收端緩衝固定 200 bytes，太長的行整行丟掉直到換行；CRC 錯、格式錯都記為壞包。
- 連續 5 次逾時判定斷線（`LINK_DOWN`），連續 3 次成功判定恢復（`LINK_UP`）。
- 結束一輪時，雙方都停止送新資料，Mega 等到自己沒有待回 ACK、而且 STM32 回報
  已停止且沒有待回 ACK，才印最終報告（`drain=confirmed`）。等不到 2 秒就標
  `NOT CONFIRMED`。
- 這裡的 100 ms 只是這個測試自己的節奏，**不是** ELRS 上下行頻率的設計。

## 編譯（不會連接或燒錄任何硬體）

```bash
python3 tools/stm32_mega_uart_test/build.py
```
先跑協定的電腦端測試（含 60 秒模擬、掉包、壞包、晚到 ACK、斷線恢復），
通過才交叉編譯，產出 `dist/stm32f401cc_mega_uart_test.bin`，並檢查
向量表、堆疊、映像大小、沒有未定義符號。結果在 `dist/verification.json`。

```bash
pio run -e mega_uart_link_test
```

## 燒錄 STM32（全新板子 → 用晶片內建的 ROM DFU，位址 0x08000000）

這個映像從 flash 開頭 `0x08000000` 開始，**不假設**有 WeAct HID bootloader
（舊專案的 `0x08004000` 映像不適用這塊板子）。燒錄會覆蓋 flash 開頭。

1. 用 USB-C 把 STM32 接到電腦。
2. 按住 **BOOT0**，按一下 **NRST** 再放開，然後放開 BOOT0。
3. 確認有進入 DFU：
   ```bash
   dfu-util -l
   ```
   應該看到 `0483:df11` 的 `@Internal Flash /0x08000000/...`（alt 0）。
   看不到的話重做第 2 步；有些板子要在插 USB 時就按住 BOOT0。
4. 燒錄並啟動：
   ```bash
   dfu-util -a 0 -s 0x08000000:leave -D tools/stm32_mega_uart_test/dist/stm32f401cc_mega_uart_test.bin
   ```
   沒有自動開始跑的話按一下 NRST。

## 燒錄 Mega 並開監視器

```bash
pio run -e mega_uart_link_test -t upload
```
```bash
pio device monitor -e mega_uart_link_test
```
開機會看到 `$ULINK_BOOT ...`。指令（打完按 Enter）：

| 指令 | 作用 |
|---|---|
| `RUN,60` | 跑 60 秒後自動停止、排空、印最終報告 |
| `START` | 不限時間開始（拔線測試用） |
| `STOP` | 停止、排空、印最終報告 |
| `STATUS` | 立刻印目前計數 |

## 驗收步驟

1. **雙方上電**：Mega 監視器在 1 秒內應出現 `$ULINK_EVENT,STM32_STATUS_RECEIVED`。
   這只代表 STM32→Mega 這個方向有東西進來，還不算通過。
2. **連跑 60 秒**：輸入 `RUN,60`。應看到 `RUNNING (STM32 confirmed)`、
   每秒一行 `$ULINK,...`、最後是報告。通過的報告長這樣：
   - 兩個方向各約 600 次，`acked` 等於 `sent`、`received` 等於對方的 `sent`
   - 所有 timeout / lateAck / dupAck / badAck / lost / dup / bad / overflow 為 0
   - `drain=confirmed`
   - `RESULT: PASS`
3. **斷線/恢復**：輸入 `START`，跑幾秒後拔掉一條訊號線
   （例如 D16→PA10）。應出現 `LINK_DOWN`，逾時數持續增加。接回後應出現
   `LINK_UP`，逾時數不再增加。再輸入 `STOP` 看報告（這次因為刻意斷線，
   `RESULT: FAIL` 是預期的，重點是逾時數和恢復）。在拔線狀態下 `STOP`，
   報告會標 `NOT CONFIRMED`。
4. 把最終報告（含 `$ULINK_RESULT,...` 那行）貼回來給 Claude/Codex。

STM32 的 LED（PC13）只是輔助：慢閃 = 沒在跑、恆亮 = 跑著且 ACK 有回、
快閃 = 跑著但斷線。**LED 不能當成通過的證據**，以 Mega 印出的數字為準。

## 已驗證 / 未驗證

- 已驗證：協定電腦端測試（含 mutation check：把 CRC 檢查、丟棄到換行、
  晚到 ACK 判定、排空條件、ACK 不觸發 ACK、內容驗證分別拿掉，對應測試都會失敗）；
  STM32 交叉編譯與映像檢查；`mega_uart_link_test` 編譯。
- 未驗證：燒錄流程、實際波特率誤差（STM32 115108 baud，-0.08%）、
  電平轉換後的實際訊號、真實往返時間、真實 60 秒結果、拔線恢復。
