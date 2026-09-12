# STM32F401CC WeAct HID LED 測試

這是獨立的 LED 驗證專案。只適用於已確認有 WeAct HID Bootloader、
預設 APP 位址 `0x08004000` 的 STM32F401CC 板子。
不是 ELRS TX/RX 韌體，也不是完整晶片映像；不要放進 ELRS 網頁 UPDATE。

## 成果與驗收

- 要選的檔案：`dist/stm32f401cc_hid_led.bin`。
- PC13 使用低電位點亮，三次短閃，接著停頓，持續重複。
- 每次亮 150 ms，前兩次各熄滅 150 ms，第三次後熄滅 1200 ms。
- 一組名義週期：`3 × 150 + 2 × 150 + 1200 = 1950 ms`。
- 內部 HSI 16 MHz，SysTick reload = `16000000 / 1000 - 1 = 15999`；
  以輪詢計時，無中斷。HSI 誤差與程式開銷會影響實際時間，並非精密時基。
- 觀察 PC13 使用者 LED，不是紅色 PWR 電源燈。
- 沒有 USB CDC、沒有 UART/CRSF 資料，也不會證明 ELRS 已配對。

## 記憶體與啟動

- 依使用者提供的 STM32F401CCU6 標示，以 256 KiB Flash / 64 KiB RAM 限制。
- Bootloader 範圍 `0x08000000..0x08003FFF`；此檔案不含該區資料。
- APP 起點 `0x08004000`，最大 240 KiB，即 `256 - 16`。
- 向量表、链接載入位址與 SCB->VTOR 一致，初始 MSP = `0x20010000`。
- 啟動碼禁止中斷、初始化資料/BSS；main 清除 NVIC/SysTick 狀態、
  切回 HSI，重設 USB 和 GPIO A/B/C 的繼承狀態，只配置 PC13 輸出。
- 此測試要求外部主電源關閉、STM32 對外線路隔離，只以 USB 供電。
- HID 工具曾回報 `V1.2 0x433 ROM:384KB`，與標示容量有差異；
  本專案沒有依該數字增加容量，也沒有驗證真實晶片全部 Flash。

## 編譯（不會燒錄）

需要 Python 3.9+、ARM GNU GCC。已驗證編譯器為
PlatformIO `toolchain-gccarmnoneeabi@1.120301.0`，GCC 12.3.1。

```sh
python3 build.py
```

可用 `ARM_GCC` 指定編譯器路徑；預設先找 PATH，再找
`~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gcc`。
這個最小專案不用 PlatformIO Upload、HAL 或下載其他韌體框架。
`build.py` 編譯並檢查向量位址、Thumb 指標、映像大小、MSP 與未解析符號，
產出 ELF、BIN、map、反組譯、section 表與 `verification.json`。

## Windows / Parallels 操作（實際寫入前先審閱）

1. 保持外部主電源關閉；確認 WeAct HID 在 Parallels 交給 Windows。
2. `MCU Info.` 應能找到 `[0483:572A]` 並讀回 V1.2。
3. 把 `dist/stm32f401cc_hid_led.bin` 複製到 Windows，
   雙擊工具左下角檔案框，選這個檔案。
4. 先核對選定檔名並保留 MCU Info 畫面。不要按 APP Erase。
5. 決定取代既有 APP 後，才按 `Download`；保留 `RebootAf.Flash` 勾選。
6. 若成功並重新啟動，觀察至少三組「三短閃＋停頓」。
   HID 裝置消失可能是 APP 已啟動，因為此 APP 沒有 USB 裝置功能。
7. 若燒錄報錯，保留完整日誌，不改試其他位址或進行整片清除。

**備份限制：** 已見的官方 GUI 只有 MCU Info、MCU Reboot、APP Erase、Download，
尚未確認可讀回 APP 的功能，目前没有既有應用程式備份。
Download 會取代既有 APP；保留 Bootloader 不等於保留舊 APP。
檔案的記憶體布局避免涵蓋 Bootloader，但尚未驗證此閉源工具實際寫入行為。

## 回到 HID

依 WeAct 說明，按住 KEY 再重新供電／NRST，看到 C13 閃爍後放開。
不要進入 HID 後再次長按 KEY，該動作會嘗試跳到系統 Bootloader。
復原成功以 `[0483:572A]` 枚舉及 MCU Info 成功回應為準。
若無法恢復，先保留狀態和現象，不用 APP Erase 當作修復。

## 已驗證與待驗證

編譯、靜態位址與映像結構檢查已完成。未燒錄、未觀察真實 LED，
未驗證上電重啟、回到 HID 或外部電路。原有兩份參考程式與 ELRS 設定未更改。

來源：
- https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1/tree/master/Soft/WeAct_HID_FW_Bootloader
- https://www.st.com/resource/en/reference_manual/rm0368-stm32f401xbc-and-stm32f401xde-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
