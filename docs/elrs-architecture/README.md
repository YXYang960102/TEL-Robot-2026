# STM32／ELRS 雙向測試：從函式到訊號

版本基準：2026-09-13，本地 `tools/stm32_elrs_bidirectional_test`。這份文件依目前原始碼解釋，並非只轉述 README。它與先前 PC13 三連閃 LED 測試是兩個不同 APP。Claude 已建立此程式；既有 verification.json 標記 flashed=false、hardware_tested=false，故「程式完成」不等於「實機已通」。

- [互動系統架構圖（Archify）](system.html)
- [互動計算與模擬教室](simulation.html)
- [原始碼](../../tools/stm32_elrs_bidirectional_test/src/main.c)
- [協議實作](../../tools/stm32_elrs_bidirectional_test/src/crsf.c)
- [來源快照雜湊](source-manifest.json)

Archify 圖中文字使用繁體中文；該版本固定 Viewer UI 與 html lang 回退為英文。圖的兩排 STM32 節點都代表同一顆實體 MCU 的不同職責，不是四顆 MCU。

## 1. 到底在測什麼

一顆 STM32 同時扮演「遙控器」與「接收側資料來源／觀察器」。主迴圈自己產生數字，UART 把數字編成 CRSF byte stream，ES900TX／ES900RX 用真正 RF 搬運資料，STM32 再讀回。

下行：`tx_ch_us → CRSF RC frame → PA9 → ES900TX → 無線 → ES900RX → PA3 → USART2 IRQ → 通道比對`。

回傳：`假電池 → CRSF Battery frame → PA2 → ES900RX → 無線 telemetry → ES900TX → PA9 → USART1 IRQ → Battery 計數`。

這兩條是各自排程的資料流。收到 RC 後「沒有」立刻回送原封包，也沒有 request ID、ACK、序號、重送佇列或往返延遲計算。假電池每秒獨立產生；目前應稱雙向鏈路觀察測試，不能稱任意封包 echo 測試。

三種測試層次：

| 層次 | 輸入／輸出 | 證明什麼 | 不能證明什麼 |
|---|---|---|---|
| Host 單元測試 | Mac 內的 C 陣列 → 真正 crsf.c → 陣列 | 既有樣本的轉換、打包、解析 | UART 時序、ELRS RF、接線 |
| 本文件互動模型 | 滑桿／時間 → JavaScript 數值與燈號 | 幫助理解程式規則 | 不執行 STM32 指令，不是 HIL |
| STM32 bench APP | 真 UART → 真 ELRS 無線 → 真 IRQ | 接線與鏈路的實際行為 | 本次尚未執行，不能由編譯成功替代 |

## 2. 系統邊界、電源與腳位

| 介面 | 本程式設定 | 角色 |
|---|---|---|
| PA9／USART1 | AF7，400000 baud，8N1，HDSEL=1 | 一條資料線輪流 TX／RX，接 ES900TX JR Signal |
| PA2／USART2 TX | AF7，設定 420000 baud | 送假電池給 ES900RX 的 RX |
| PA3／USART2 RX | AF7，設定 420000 baud | 接收 ES900RX TX 的 RC frame |
| PC13 | 一般輸出，低電位亮 | 顯示軟體推定的鏈路狀態 |
| USB | APP 啟動時重設 USB 周邊 | Bootloader 燒錄入口／供電；APP 沒有 USB CDC |

程式沒有讀取搖桿 ADC、沒有把 USB 資料轉 UART、沒有網頁服務，也沒有控制馬達。`1500 us` 是 PWM 習慣使用的通道數值單位，不代表 PA9 上會輸出 1500 us PWM 脈波。PA9 送的是 UART 位元。

程式把 CH4／CH5／CH6／CH8 設為 1000，註解稱 Fire off／Arm off／Manual／Emergency off；這只是測試作者假定的通道定義，不能推論所有外接控制器在此值都安全。桌上測試應只連已核對的通訊硬體。ES900TX 電源獨立供應，共地；其規格與腳位需以實際模組為準。

ELRS 射頻封包格式、跳頻、重試、telemetry 排程由模組韌體負責，這份 STM32 原始碼沒有實作它們。10 ms 是送入 TX 的 UART 頻率，不能直接當作空中 packet rate。

## 3. 檔案與誰負責哪一層

| 檔案 | 職責 | 誰呼叫／使用 |
|---|---|---|
| src/config.h | 位址、時脈、baud 暫存器、週期、容差 | main.c 編譯時常數 |
| src/startup.S | 向量表、重設入口、資料初始化、預設異常處理 | Bootloader 跳 APP、CPU 中斷向量 |
| linker.ld | 把向量、程式、初始化資料、BSS 放到地址 | ARM linker |
| src/main.c | 暫存器初始化、時間排程、收送、判定、LED | Reset_Handler／中斷硬體 |
| src/crsf.h | 公開 API、常數、CrsfParser／CrsfFrame | main.c、crsf.c、host test |
| src/crsf.c | 純資料轉換、CRC、pack/unpack、串流解析 | main 與 IRQ、host test |
| test/host_test_crsf.c | 六組 CPU 本地單元測試 | build.py |
| build.py | 先測試再交叉編譯、靜態檢查、產物紀錄 | 使用者 `python3 build.py` |
| dist/* | BIN／ELF／map／反組譯／驗證 JSON | 燒錄工具與工程檢查 |

## 4. 啟動與記憶體：CPU 如何找到 main

APP 起點 `0x08004000`。保留 Bootloader 前 16 KiB；256 KiB −16 KiB=240 KiB=245760 bytes。APP flash 範圍 `0x08004000..0x0803FFFF`。RAM `0x20000000..0x2000FFFF`，初始 MSP 為上緣 `0x20010000`，stack 往下成長。

向量表 word index 0 是 MSP，1 是 Reset_Handler，15 是 SysTick，53=16+37 是 USART1，54=16+38 是 USART2。每個 word 4 bytes，例如 USART1 向量槽地址 `0x08004000+53×4=0x080040D4`；槽內存的是 handler 位址，不是把 handler 程式放在槽內。Thumb 函式指標 bit0=1。

Reset_Handler 順序與分支：

1. `cpsid i` 遮罩一般中斷，設定 MSP、CONTROL=0。
2. 比較 `_sdata.._edata`：還有 word 就從 Flash `_sidata` 複製到 RAM，結束進下一段。`tx_ch_us` 的初值靠此步建立。
3. 比較 `_sbss.._ebss`：還有 word 就清零。g_millis、parser、狀態旗標因此初始為零。
4. `bl main`。若 main 意外返回，跳 Default_Handler。
5. Default_Handler 關中斷並原地無限迴圈。未配置的例外大多指向此處，沒有錯誤編碼或自動復原。

linker 將 `.isr_vector` KEEP 在 FLASH 起點；`.text/.rodata` 在 FLASH；`.data` 執行地址在 RAM、初值儲存地址在 FLASH；`.bss` 不佔 BIN 的資料內容。兩個 ASSERT 檢查 APP 起點與 BSS 結尾至少保留 4096 bytes stack 空間，後者不是執行期 stack 水位量測。

## 5. main.c 每個函式與呼叫關係

| 函式 | 輸入／輸出與副作用 | 呼叫誰／關鍵分支 |
|---|---|---|
| prepare_hardware | 無參數；重設繼承狀態、HSI、GPIOC、SysTick | 8 次停用／清除 NVIC；等 HSI ready、系統時脈切換、PLL off，三處均無 timeout |
| usart_gpio_init | 設 PA9／PA2／PA3 為 AF7、push-pull、上拉、高速 | 無子函式；每 pin MODER 2 bits、AFR 4 bits |
| usart1_init | 開 APB2 USART1 clock；BRR=0x28、HDSEL=1 | 初始 UE+RE+RXNEIE，先收 |
| usart2_init | 開 APB1 USART2 clock；BRR=0x26 | UE+TE+RE+RXNEIE，全雙工 |
| nvic_enable_usarts | NVIC_ISER1 bit5、bit6 置 1 | IRQ37−32=5、IRQ38−32=6 |
| usart1_switch_to_tx | 關 RE、開 TE | send_rc_to_tx_module 呼叫 |
| usart1_switch_to_rx | 關 TE、開 RE | 等整包 TC 後呼叫 |
| send_rc_to_tx_module | 建 26-byte 區域 frame；寫 USART1_DR | build_rc → switch TX → 逐 byte 等 TXE → 等 TC → switch RX |
| send_battery_to_receiver | static 假電壓遞增；寫 USART2_DR | 超過126回120；build_battery；逐 byte 等 TXE，末尾不等 TC |
| channels_match_sent | 兩個 int[16] → bool | 只看 index0..7；差值取絕對值，任一 >20 回 false，否則 true |
| update_demo_channels | 修改 tx_ch_us[0..7] | state=(state+1)%3；三個 if 設 CH1；其餘固定 |
| led_on | PC13 BSRR reset bit29=1 | 13+16=29，輸出低電位 |
| led_off | PC13 BSRR set bit13=1 | 輸出高電位 |
| update_status_led | 讀狀態時間，控制 LED | 雙向 true 恆亮 return；否則選 200/1000 ms，再比較半週期 |
| SysTick_Handler | g_millis++ | CPU 每 nominal 1ms 中斷呼叫，不由 main 直接呼叫 |
| USART1_IRQHandler | SR／DR byte → parser_tx_module → 狀態 | RXNE 優先；完整 frame 更新時間；type=Battery 才加 count；否則 ORE 清 DR |
| USART2_IRQHandler | SR／DR byte → parser_receiver → RC數值 | RC type 且payload22才 unpack／換算／比對／更新時間；否則忽略；ORE 分支清 DR |
| main | 初始化後永久忙迴圈 | prepare → GPIO → USART1/2 → parser_init兩次 → NVIC → cpsie i；每圈3個獨立排程 if、LED |

main 沒有 RTOS、sleep、DMA、ring buffer 或工作佇列。收資料由 IRQ 插入主流程，送資料用 blocking polling。`TXE` 表示可再寫下一個 byte，`TC` 表示最後 byte（含 stop bit）發完；半雙工不能只等 TXE 就換回接收。

## 6. 時間、時脈與排程：實際怎麼算

HSI=16000000 Hz；AHB/APB prescaler 清為1，兩個 UART 取得16MHz。SysTick reload `16000000/1000−1=15999`，計數16000個 clock一次，nominal1ms。HSI 實際誤差會同時影響 millis 與 baud。

16倍oversampling下，USART baud=`PCLK/(16×mantissa+fraction)`：

- BRR=0x28 → mantissa2、fraction8 → denominator40 → 400000 baud。
- BRR=0x26 → mantissa2、fraction6 → denominator38 → 421052.63 baud；相對420000為 `(421052.63−420000)/420000×100%=+0.2506266%`。
- config.h 註解把理想 USARTDIV 寫成2.375；嚴格說理想值為 `16000000/(16×420000)=2.380952`，2.375是量化後的值。程式 BRR 設定與上述實際 baud 一致。

8N1每byte=1 start+8 data+1 stop=10 bits。RC26 bytes →260bits／400000=650us；每10ms送一包，單就RC的線上佔用約6.5%。電池12 bytes →120bits／421052.63≈285us，每秒一次。這不包含 RF 延遲、IRQ 額外時間或模組回傳窗口。

排程 if 都是 `(uint32_t)(now-last)>=period`：到期才做，並設定last=now。它們不是 else-if；同一圈可依序做 demo、RC、Battery。若主迴圈拖延，沒有補送所有漏掉的週期，只做一次；不是精準硬即時100Hz保證。

32-bit millis 約49.7103天回捲。unsigned減法可跨單次回捲算短間隔，例如 now=5、last=0xFFFFFFFE，差=7ms。長時間不更新的歷史旗標在整個32-bit週期後仍可能有陳舊重合，不能當永久狀態資料庫。

### CH1 實際時序（與 README 不同）

| 啟動時間 | state | CH1 |
|---|---|---|
| 0到2秒 | static初始0但尚未呼叫 | 陣列初始1500 |
| 2到4秒 | 第一次先加成1 | 1500 |
| 4到6秒 | 2 | 2000 |
| 6到8秒 | 0 | 1000 |
| 8到10秒 | 1 | 1500 |

因此不是一開機1000→1500→2000。完整穩態週期6秒；第一次RC約10ms，第一次Battery約1000ms。每次demo更動完成後，當圈若RC也到期，就送新值。

## 7. 資料結構、所有重要狀態

| 名稱 | 類型／容量 | 初值、所有者與用途 |
|---|---|---|
| tx_ch_us | int[16]，ARM通常64 bytes | {1500,1500,1500,1000,1000,1000,1500,1000,1500×8}；main更新，USART2 IRQ讀 |
| last_rx_ch_tick | uint16_t[16]，32 bytes | IRQ2存解析後11-bit數值 |
| last_rx_ch_us | int[16]，通常64 bytes | IRQ2換算後的通道；沒有輸出到USB |
| CrsfParser | buf[64]+index+expected_len，66 bytes | 每條UART一份；index是已收byte數；expected_len是線上length欄位 |
| CrsfFrame | address,length,type,payload_len各1byte+payload[64]，68 bytes | IRQ stack 上的完整frame；合法payload實際最大60bytes |
| g_millis | volatile uint32_t | SysTick寫；main與IRQ讀，nominal毫秒 |
| downlink_ok | volatile bool | 最近一次有效RC與目前期望值的比對結果 |
| last_rc_rx_ms | volatile uint32_t | 每次有效RC都刷新，即使比對失敗 |
| last_txmodule_frame_ms | volatile uint32_t | USART1每個CRC正確frame刷新，並不限Battery |
| battery_backlink_count | volatile uint32_t | 每個Battery型別+1，不驗payload長度或內容 |
| last_*_send_ms／last_demo_ms | main區域 uint32_t | 排程上次執行時間，不是接收時間 |
| fake_voltage_01v | 函式static uint16_t | 初始120，每次先加1，>126回120 |
| state | 函式static uint8_t | 0..2的CH1模式索引 |

`volatile`避免讀寫被當普通不變記憶體最佳化；不會讓多欄位快照變成原子交易。`tx_ch_us` 本身不是 volatile，也沒有更新鎖或雙buffer；IRQ能插入多通道更新中間。這份測試只有CH1明顯變動，但若擴充多通道同步更新，應另設快照／臨界區。

## 8. crsf.c 每個函式、演算法與數值例子

### crsf_us_to_tick(us) → uint16_t

先clamp us到988..2012，再算 `tick=trunc((us−1500)×8/5)+992`，最後clamp到0..2047。C signed整除向零截斷。

- 1000 → (−500×8/5)+992=192；1500→992；2000→1792。
- 988→trunc(−4096/5)+992=173；2012→1811。
- 輸入1會先被clamp成988，故173。不要把常見協議端點172混作這份公式在988的結果。

### crsf_tick_to_us(tick) → int

`trunc((tick−992)×5/8)+1500`，沒有clamp。192→1000，992→1500，1792→2000。173→989，表示round-trip允許1us誤差。若直接傳入超過11bit的數值，此函式也照算；呼叫者要負責輸入契約。

### crsf_pack_channels(ch[16],payload[22])

先清22 bytes。每通道取 `ch[i]&0x07FF`，依序放進32-bit bitbuf 的低位累積區；bits+=11。只要bits>=8，就取最低byte輸出、bitbuf右移8、bits-=8。16×11=176bits=22bytes，剛好無剩餘。

CH1=192=0x0C0、CH2=992=0x3E0、CH3=992時，前3 payload bytes為 `C0 00 1F`。CH1 bits0..7先出C0；剩3bits與CH2低5bits組下一byte。這是跨byte的LSB-first位元打包，不是每通道直接佔2bytes。最大待用bits在加入一通道後不超過18，32-bit容器足夠。

### crsf_unpack_channels(payload[22],ch[16])

bits不足11且in<22才補byte到bitbuf；每次取低11bits輸出，右移11、bits-=11。固定22-byte輸入才符合契約；此函式本身不接受實際長度，IRQ2先查payload_len=22。pack/unpack是反向操作。

### crsf_crc8(data,len)

crc初始0；每byte先XOR，再重複8次：bit7=1則左移並XOR0xD5，否則只左移；每次收斂到uint8_t。CRC只含type+payload，排除address、length與CRC自身。只有type=0x16的單byte範例：16→2C→58→B0→B5→BF→AB→83→D3，因此CRC8([16])=D3。CRC是錯誤偵測，不是加密或來源驗證。

### crsf_build_rc_frame(frame[26],ch_us[16])

呼叫16次us_to_tick，再pack，最後crc8。布局如下：

| index | 值／內容 |
|---|---|
| 0 | 0xEE，CRSF_TX_ADDR，編譯時可覆寫 |
| 1 | 24=type1+payload22+CRC1，不含前2bytes |
| 2 | 0x16，RC Channels |
| 3..24 | 22-byte payload |
| 25 | CRC8(frame[2..24])，23bytes |

### crsf_build_battery_frame(...) → 12

address C8、length10、type08；電壓與電流16bit big-endian，容量只取低24bits big-endian，百分比1byte未clamp。輸入電流5表示0.5A，容量100表示100mAh，百分比90表示90%。容量>0xFFFFFF會截掉高位；百分比>100不會拒絕，這些是API邊界。

第一次send先把電壓120加成121，所以送12.1V，不是12.0V。其後12.2、12.3、12.4、12.5、12.6、12.0循環，每7次送出重複。

| index | 第一次送出的資料（CRC由程式計算） |
|---|---|
| 0..2 | C8 0A 08 |
| 3..4 | 00 79（121×0.1V=12.1V） |
| 5..6 | 00 05（0.5A） |
| 7..9 | 00 00 64（100mAh） |
| 10 | 5A（90%） |
| 11 | CRC8(index2..10，共9bytes) |

### crsf_is_possible_address(b) → bool

只接受00/C8/EA/EC/EE；switch任一命中true，default false。这是此解析器的白名單，不代表完整CRSF所有地址集合。

### crsf_parser_init(p)

index=0，expected_len=0，清64bytes。main對兩個獨立parser各呼叫一次；IRQ1和IRQ2不会共用同一串流状态。

### crsf_parser_push(p,b,out) → bool：每個分支

1. index=0時檢查地址，不在白名單就false，未存byte。
2. 存 `buf[index++]`。
3. 收到第2byte時取length；若<2或>62，清index/expected_len，false。目前byte不会重作新地址。
4. index>=2且expected_len>0才看完整性。total=length+2，未收到total就繼續等待，false。
5. 收齊後CRC計算length−1bytes（type+payload）。若等於最後byte，填out，payload_len=length−2，複製payload，重設parser，true。
6. CRC不同：重設，false，不回溯搜尋已吃掉的內部bytes。
7. 防禦分支index>=64則重設，false；合法長度通常在收齊分支已處理。

false同時表示「还没收齐」「无效地址」「坏长度」「CRC错」，沒有错误枚举或统计。沒有字节间timeout。若噪声刚好是允许地址+合法长度，会吞後续byte直到凑满；不能把目前简单垃圾前缀测试称为所有串流破坏都能立即恢复。

## 9. 接收 IRQ 每個分支如何改變結果

USART1：讀SR → RXNE？是則讀DR低8bits → push。未完整／CRC錯不更新。完整frame不論type都刷新last_txmodule_frame_ms；只有type08才增加battery_backlink_count。其他type雖然不加count，仍使時間變新。RXNE不成立才檢查ORE；ORE成立讀DR清除。RXNE與ORE同時有值時走前者，SR後讀DR也用於清溢位，但沒有記錄遺失byte數。

USART2：同樣RXNE優先 → push；完整且type16且payload22才unpack16通道→逐一tick_to_us→比對前8→設定downlink_ok→刷新last_rc_rx_ms。其他type或錯誤長度不刷新RC時間。ORE分支同上。Parity／Framing／Noise其他錯誤沒有獨立計數或明確處置。

通道比對取絕對值，≤20us接受：送1500、回1519成功；回1520也成功；回1521失敗。任一CH1..8失敗立刻false；CH9..16即使不同也不影響判定。

比的是「IRQ當下的期望陣列」，不是帶序號的歷史送出封包。CH1剛切換時，舊RF資料尚在路上，可能短暫不匹配；下一包新值才恢復。這不等於立刻證明RF斷線。

## 10. LED 判斷真值表與容易誤讀的地方

`D = downlink_ok && (now-last_rc_rx_ms <500)`。
`B = (battery_backlink_count>0) && (now-last_txmodule_frame_ms <1500)`。

| D | B | 行為 |
|---|---|---|
| false | false | 慢閃，1000ms週期，亮500/暗500 |
| false | true | 同樣慢閃；看不出回傳單獨正常 |
| true | false | 快閃，200ms週期，亮100/暗100 |
| true | true | 恆亮 |

判斷為嚴格 `<`：RC age499有效，500失效；回傳age1499有效，1500失效。phase=now%period，phase<period/2亮，否則暗。開機now很小時D=false也會先亮半個慢閃週期，不能看到瞬間亮就判成功。

重要限制：B的時間戳是「任意CRC正確frame」！曾收過一包Battery後，只要持續有其他合法frame，B也可能一直true，即使後來電池回傳已停止。並且type08沒有驗證payload_len=8、電壓內容、序號。LED恆亮只證明目前這組程式條件成立，不能證明每秒假電池被原樣回傳。

最小修正建議（尚未修改韌體）：只在Battery型別+正確長度+期望內容命中時更新專用last_battery_rx_ms；如需真正往返測試，另設可追蹤序號與發送時間、逾時／丟包計數。不要把電池單位欄位任意改作未定義封包而仍假定ELRS會轉送。

## 11. Host 模擬怎麼呼叫、涵蓋每個測試

在專案根目錄以原生C編譯器執行，不連USB：

```sh
gcc -Wall -Wextra -Werror -std=c11 tools/stm32_elrs_bidirectional_test/test/host_test_crsf.c tools/stm32_elrs_bidirectional_test/src/crsf.c -o /tmp/elrs-doc-host-test
/tmp/elrs-doc-host-test
```

host `main()`依次呼叫：

| 測試函式 | 方法／分支 |
|---|---|
| test_tick_round_trip | 988/1000/1500/1750/2012/1，正反換算與clamp後差≤1；1500=992 |
| test_channel_pack_round_trip | ch[i]=172+i×100；pack→unpack，16值完全相等 |
| test_rc_frame_parses_back | ch_us[i]=1000+i×50；build→逐byte push→unpack→換算，差≤1 |
| test_battery_frame_parses_back | 126,5,1234,77；檢查長度／地址／type／解析電壓126 |
| test_parser_rejects_bad_crc | 26-byteRC最後CRC XOR FF；每byte推入，全程不得true |
| test_parser_resyncs_on_garbage_prefix | 前綴11/22/33三個非地址；後接合法RC可解析 |

CHECK失敗印訊息並累加failures，仍繼續測；最後failures=0印All CRSF host tests passed，return0，否則return1。部分測試在前置解析失敗後仍讀out，沒有fail-fast，不應把任意失敗時的後續輸出當可靠診斷。

這六組會重用同一crsf.c去生成和解析，可能共同接受同一種格式錯誤。沒有獨立已知正確CRSF golden vector，也沒有測所有bad-length、截斷、嵌入合法地址的噪聲、IRQ時序、LED stale或假電池內容關聯。本次原生測試已重新跑過並通過；不擴大宣稱測試覆蓋率。

## 12. build.py 所有階段／失敗分支

此檔為頂層Python流程，沒有自訂def；呼叫外部工具，任何subprocess check=True失敗即停止。

1. 建dist。找HOST_CC，否則gcc，否則cc；都無則SystemExit。TemporaryDirectory內編host test，執行後自動清臨時檔。
2. 找ARM_GCC，否則PATH，否則PlatformIO固定候選；都無則SystemExit。
3. ARM GCC用Cortex-M4、Thumb、soft float、Os、freestanding、nostdlib、nostartfiles；linker.ld決定布局。警告作錯誤，無HAL／標準C runtime依賴。
4. objcopy ELF→raw BIN；objdump產section和disassembly。
5. 讀BIN首8bytes驗MSP=20010000、reset Thumb bit、reset落在映像內、總長≤245760。
6. 搜.isr_vector，必須VMA=LMA=08004000；找不到assert失敗。
7. 解102個向量；除MSP外都必須Thumb且映像內。15/53/54不等預設向量，53不等54。此檢查確認不是Default且彼此不同，但沒有用符號表逐一比對handler名稱，不能宣稱完整語義驗證。
8. nm -u不得有未解析符號。
9. report記compiler、位址、向量、size、sha256，hardware_tested=false與flashed=false為硬寫入的建置狀態；不是自動偵測硬體。

本次讀到既有BIN為2384bytes，SHA256 `0b3c43a945cafafc32df46589bd8774b0aa08d42983b27d71c243427d7a67ac6`。本次為文件工作，只重跑host test，未重新產出韌體，也未燒錄。互動模型和圖不修改此BIN。

## 13. 實體驗證應如何分階段觀察

1. 核對實際接線、共地、獨立TX供電、ELRS離開WiFi設定模式；先前截圖兩端3.2.0/FCC915/UID一致是歷史設定證據，非此刻連線證據。
2. 若另行進行已授權的燒錄，APP載入位址必須08004000，不可把本APP寫到08000000。本文不執行燒錄。
3. 量PA9：應約10ms一組26bytes，baud400000；檢查非反相8N1與發完TC才換收。
4. 量PA3：應能看到有效RC frame；解CH1長時間序列，先1500約4秒，再2000/1000/1500每2秒。
5. 量PA2：每秒12bytes假電池，首包12.1V；PA9回傳應另解type08與8-byte內容。
6. 人為停止下行／回傳，量測各自失效時間；目前LED的Battery stale缺陷要以資料記錄揭露，不能以恆亮代替測量。

此程式阻塞等待硬體ready/TXE/TC時沒有timeout；中斷內解析與CRC會占CPU；HSI誤差、半雙工切換／模組回傳撞期、GPIO push-pull與外部線路電氣條件都需實測。不能只靠host test通過保證硬體可用。

## 14. 下一階段可以延伸的問題

- 需求是RC+telemetry還是任意byte echo？若後者，是否選AirPort或另一明確支援的傳輸模式？
- 想測「收到類型」「內容一致」「每包送達」「延遲」「失聯復原」哪一層？定義各自通過條件。
- 回傳是否有序號／時間戳／CRC之外的內容比對？如何區分舊包、新包、重複包？
- 主迴圈與IRQ共享資料是否需要快照？是否需要ring buffer與主迴圈解析降低IRQ負担？
- 是否要加入timeout、錯誤計數、last_good與last_any分開、USB/UART debug觀察？
- ELRS包率、switch mode、telemetry ratio能否保留前8通道所假定的精度？先觀察真實輸出再訂容差。
- 哪些是注釋／設計意圖，哪些已由測試、反組譯或實機log證明？每次交接應保留這個差別。

上述是後續設計議題，不表示此次已修改功能。

## 15. 函式來源索引

以下行號依本次來源快照產生；圖與說明沒有改動來源。

| 檔案／函式 | 定義行號 |
|---|---|
| [main.c：prepare_hardware](../../tools/stm32_elrs_bidirectional_test/src/main.c#L92) | 92 |
| [main.c：usart_gpio_init](../../tools/stm32_elrs_bidirectional_test/src/main.c#L139) | 139 |
| [main.c：usart1_init](../../tools/stm32_elrs_bidirectional_test/src/main.c#L155) | 155 |
| [main.c：usart2_init](../../tools/stm32_elrs_bidirectional_test/src/main.c#L166) | 166 |
| [main.c：nvic_enable_usarts](../../tools/stm32_elrs_bidirectional_test/src/main.c#L176) | 176 |
| [main.c：usart1_switch_to_tx](../../tools/stm32_elrs_bidirectional_test/src/main.c#L184) | 184 |
| [main.c：usart1_switch_to_rx](../../tools/stm32_elrs_bidirectional_test/src/main.c#L188) | 188 |
| [main.c：send_rc_to_tx_module](../../tools/stm32_elrs_bidirectional_test/src/main.c#L196) | 196 |
| [main.c：send_battery_to_receiver](../../tools/stm32_elrs_bidirectional_test/src/main.c#L209) | 209 |
| [main.c：channels_match_sent](../../tools/stm32_elrs_bidirectional_test/src/main.c#L222) | 222 |
| [main.c：update_demo_channels](../../tools/stm32_elrs_bidirectional_test/src/main.c#L231) | 231 |
| [main.c：led_on](../../tools/stm32_elrs_bidirectional_test/src/main.c#L253) | 253 |
| [main.c：led_off](../../tools/stm32_elrs_bidirectional_test/src/main.c#L254) | 254 |
| [main.c：update_status_led](../../tools/stm32_elrs_bidirectional_test/src/main.c#L256) | 256 |
| [main.c：SysTick_Handler](../../tools/stm32_elrs_bidirectional_test/src/main.c#L276) | 276 |
| [main.c：USART1_IRQHandler](../../tools/stm32_elrs_bidirectional_test/src/main.c#L280) | 280 |
| [main.c：USART2_IRQHandler](../../tools/stm32_elrs_bidirectional_test/src/main.c#L296) | 296 |
| [main.c：main](../../tools/stm32_elrs_bidirectional_test/src/main.c#L321) | 321 |
| [crsf.c：crsf_crc8](../../tools/stm32_elrs_bidirectional_test/src/crsf.c#L3) | 3 |
| [crsf.c：crsf_us_to_tick](../../tools/stm32_elrs_bidirectional_test/src/crsf.c#L14) | 14 |
| [crsf.c：crsf_tick_to_us](../../tools/stm32_elrs_bidirectional_test/src/crsf.c#L23) | 23 |
| [crsf.c：crsf_pack_channels](../../tools/stm32_elrs_bidirectional_test/src/crsf.c#L27) | 27 |
| [crsf.c：crsf_unpack_channels](../../tools/stm32_elrs_bidirectional_test/src/crsf.c#L45) | 45 |
| [crsf.c：crsf_build_rc_frame](../../tools/stm32_elrs_bidirectional_test/src/crsf.c#L61) | 61 |
| [crsf.c：crsf_build_battery_frame](../../tools/stm32_elrs_bidirectional_test/src/crsf.c#L74) | 74 |
| [crsf.c：crsf_is_possible_address](../../tools/stm32_elrs_bidirectional_test/src/crsf.c#L94) | 94 |
| [crsf.c：crsf_parser_init](../../tools/stm32_elrs_bidirectional_test/src/crsf.c#L107) | 107 |
| [crsf.c：crsf_parser_push](../../tools/stm32_elrs_bidirectional_test/src/crsf.c#L113) | 113 |
| [host_test_crsf.c：test_tick_round_trip](../../tools/stm32_elrs_bidirectional_test/test/host_test_crsf.c#L18) | 18 |
| [host_test_crsf.c：test_channel_pack_round_trip](../../tools/stm32_elrs_bidirectional_test/test/host_test_crsf.c#L31) | 31 |
| [host_test_crsf.c：test_rc_frame_parses_back](../../tools/stm32_elrs_bidirectional_test/test/host_test_crsf.c#L43) | 43 |
| [host_test_crsf.c：test_battery_frame_parses_back](../../tools/stm32_elrs_bidirectional_test/test/host_test_crsf.c#L74) | 74 |
| [host_test_crsf.c：test_parser_rejects_bad_crc](../../tools/stm32_elrs_bidirectional_test/test/host_test_crsf.c#L95) | 95 |
| [host_test_crsf.c：test_parser_resyncs_on_garbage_prefix](../../tools/stm32_elrs_bidirectional_test/test/host_test_crsf.c#L112) | 112 |
| [host_test_crsf.c：main](../../tools/stm32_elrs_bidirectional_test/test/host_test_crsf.c#L133) | 133 |
