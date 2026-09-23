# Jetson Orin Nano ↔ Arduino Mega 2560 UART 完全無訊號問題

## 硬體配置
- **Jetson Orin Nano**（新的板子，今年剛換的，不是去年那顆）
- **Arduino Mega 2560**（跟去年同一顆板子，沒換過）
- 兩者之間接一塊 **TaiwanIOT 4路 BSS138 雙向邏輯電平轉換模組**（HV=5V / LV=3.3V / 共用GND），這塊模組本身也是去年同一顆、原封不動拆下來裝到現在這組架子上的
- 接線方式：
  - Mega D18(TX1) → 轉換板 HV 側訊號通道 → LV 側 → Orin 40-pin 排針 **pin 10**(RX)
  - Orin 40-pin 排針 **pin 8**(TX) → 轉換板 LV 側訊號通道 → HV 側 → Mega D19(RX1)
  - Orin 40-pin 排針 **pin 6**(GND) 直接接 Mega GND，兩板共地
  - 轉換板上另外標示的 `HV`／`LV` 供電腳位**沒有接**（只接了訊號通道跟共地），但這個配置去年也是這樣、當時可以正常通訊
  - Orin 用原廠 19V 電源，Mega 用電腦 USB 供電，各自獨立電源
- 目標：Orin 上的 Python 腳本（`pyserial`）跟 Mega 韌體用 UART 115200 baud、8N1 互傳文字協定（Mega 開機送 `MEGA_READY,1`、之後每 200ms 送 `MEGA_HEARTBEAT,1`；Orin 端等到心跳才會往下執行）

## 症狀
**兩個方向完全零位元組**，用最原始的方式測試都一樣：
```bash
stty -F /dev/ttyTHS1 115200 raw -echo
cat /dev/ttyTHS1
```
不管等多久、Mega 是否正在送資料，畫面完全沒有任何輸出，連一個亂碼字元都沒有。

## 已經排除、且是用實測而不是猜測排除的項目
1. **Orin 端 Python 程式邏輯**：`cat`/`stty` 完全繞過 Python，直接讀 Linux 核心 UART 驅動層的 raw bytes，兩者結果一樣是 0——代表問題不可能出在任何 Python 程式碼上。
2. **裝置節點選錯**：這台 Orin 上只存在 `/dev/ttyTHS1`(MMIO 0x3100000) 跟 `/dev/ttyTHS2`(MMIO 0x3140000) 兩個節點，兩個都測過、都是 0 byte。用 `sudo /opt/nvidia/jetson-io/jetson-io.py` 確認 40-pin 排針 pin 8/10 目前設定的功能是 `uarta`；`sudo dmesg | grep -iE "tty|uart"` 顯示 `ttyTHS1` 探測到的 MMIO 位址正好是 UARTA 的位址 `0x3100000`——所以 `/dev/ttyTHS1` 已確認是 pin 8/10 對應的正確節點，不是選錯節點的問題。
3. **40-pin 排針 UART 功能沒開**：`jetson-io.py` 顯示 pin 8/10 已經是 `uarta`，不是 `unused`，代表 pinmux 已經正確設定，不需要額外用 jetson-io 開啟。
4. **Orin 端序列埠權限問題**：一開始有 `Permission denied`，已經用 `sudo usermod -aG dialout <user>` 加入群組解決，現在 port 能正常開啟（不再報錯），只是開了之後完全收不到東西。
5. **Mega 板子本身 UART 硬體故障**：用一支完全裸機、沒接 shield、沒接任何外部裝置、沒有任何跳線短接的獨立診斷韌體，讓 Mega 的 RX1(D19) 設成 INPUT_PULLUP，先安靜監聽 10 秒（確認沒有雜訊/floating 干擾，結果 `rx_bytes=0`），再讓 Mega 自己每 200ms 對外送一次心跳 10 秒（`tx_frames=49`），同時繼續監聽 RX1，結果一樣 `rx_bytes=0`——在完全沒有外部連接的乾淨環境下這是正常/健康的結果，沒有任何異常雜訊或非預期迴授，排除 Mega 板子本身 TX1/RX1 有硬體短路或損壞。
6. **轉換板 HV/LV 供電缺失**：這是我們一度懷疑的重點，因為這類 BSS138 電平轉換 IC 理論上沒有 HV/LV 供電就不會動作。但後來確認這塊模組去年在同樣沒接 HV/LV 供電腳的情況下，本來就能正常運作，所以這不是這次的變因。

## 已知唯一「跟去年不一樣」的變數
Mega 板子、轉換板模組、線材接法、使用的排針位置(6/8/10)都跟去年「可以動」的那組完全相同。**唯一確認換掉的是 Jetson Orin Nano 本身這片板子**——是新的一片，不是去年用過的那片。目前還沒有找到「新的 Orin Nano 板子」具體是哪個環節導致完全收不到/送不出訊號，因為 pinmux 設定跟裝置節點在軟體層面看起來都正確。

## 手邊沒有電表可以做電壓/通斷量測，目前只能用替換法

## 想問的問題
在 pinmux 已確認正確(`uarta`)、裝置節點已確認正確(`ttyTHS1`)、軟體邏輯已確認正常、Mega 端硬體已確認正常、電平轉換板供電已確認不是必要條件的前提下，**一片全新的 Jetson Orin Nano（相對於去年用過、已知正常的那片）**，還有哪些可能導致 40-pin 排針 UART1(pin 8/10) 完全沒有任何訊號進出（不是格式錯誤、不是雜訊，是徹底零位元組）？特別想知道：
- 全新的 Jetson 裝置在「首次」啟用 40-pin 排針功能時，除了 `jetson-io.py` 顯示的 pinmux 設定之外，是否還有其他一次性的初始化/韌體/開機參數步驟容易被漏掉？
- 是否有已知的 Jetson Orin Nano 硬體版本差異（例如原廠 carrier board 修訂版、或近期發表的「Orin Nano Super」）可能造成 40-pin 排針上同一根實體位置的電氣特性或內部走線跟舊款不同？
- 除了電表量測，還有沒有其他不需要額外工具、能進一步縮小範圍的診斷方法？
