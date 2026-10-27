# 射擊表（Excel → Mega）

`shot_table.xlsx` 是射擊表的唯一來源。Mega 用的 `src/Constants/ShotTableData.h`
由它自動產生，**不要手改那個 .h**。

## 使用方式

1. 用 Excel 打開 `tools/shot_table/shot_table.xlsx` 的「射擊表」工作表，改數字、
   加列或刪列，存檔（保持 .xlsx）。表格旁邊有三張射擊曲線圖（下、中、上排），
   改值、加列、刪列都會自動跟著更新。
2. 在專案資料夾（`TEL_Robot_2026`）執行：

   ```bash
   python3 tools/shot_table/apply.py --upload
   ```

   它會依序：檢查格式 → 列出這次改了哪些值 → 產生 `ShotTableData.h` →
   編譯並燒進 Mega（Mega 要用 USB 接著這台電腦）。

   - 只想檢查、不寫任何檔案：`python3 tools/shot_table/apply.py --check`
   - 只產生 .h、不燒錄：`python3 tools/shot_table/apply.py`
   - Windows 上把 `python3` 換成 `py` 或 `python`。
3. 把 `shot_table.xlsx` 和 `src/Constants/ShotTableData.h` 一起 commit，隊友
   pull 下來就同步了。

不需要安裝任何 Python 套件。燒錄要有 PlatformIO：它會依序找 PATH 上的 `pio`、
VS Code PlatformIO 擴充功能自己的那份、最後是用 pip 裝進同一個 Python 的
PlatformIO（`python -m platformio`）。都沒有時會提示怎麼裝。

## Mac / Windows 指令對照

先進到專案資料夾（Mac：`cd /Users/jeremy/Desktop/FRC8169/TEL_Robot_2026`；
Windows 是你 clone 的位置，例如
`cd C:\Users\JasonPro\Desktop\TEL2026\code\YXY\TEL-Robot-2026`）。

| 做什麼 | Mac（Terminal） | Windows（PowerShell） |
|---|---|---|
| 裝 PlatformIO（只要一次；已裝 VS Code 擴充功能可跳過） | `python3 -m pip install -U platformio` | `py -m pip install -U platformio` |
| 只檢查 Excel | `python3 tools/shot_table/apply.py --check` | `py tools/shot_table/apply.py --check` |
| 看 S 紀錄會改什麼 | `python3 tools/shot_table/apply.py --import --check` | `py tools/shot_table/apply.py --import --check` |
| S 紀錄寫進 Excel + 燒 Mega | `python3 tools/shot_table/apply.py --import --upload` | `py tools/shot_table/apply.py --import --upload` |
| 只用 Excel 更新並燒 Mega | `python3 tools/shot_table/apply.py --upload` | `py tools/shot_table/apply.py --upload` |

燒錄前在 Dashboard 按斷線（不然序列埠被佔住），並關掉 Excel。

## 用 Dashboard 按 S 記錄（2026-10-26）

測試時不用手抄：

1. Dashboard 模式選單選「測試 TEST」（或全手動），Enable、按 Y。水平由鏡頭鎖定，
   仰角（十字鍵 ↑↓ 或角度輸入）、轉速（滑桿或 LT/RT）、發射都手動。
2. 按 1–9 選洞口，調好仰角和轉速、打進之後，**按 S**。Dashboard 的「Shot Log /
   射擊紀錄」會多一列：洞口、相機距離、仰角脈寬、轉速（%）。
   - 距離是最近 1.5 秒內、鏡頭看到那個洞的距離中位數（一兩格跳動的深度不會被記到）。
   - 會拒絕記錄並在 Serial Log 說原因：沒選洞口（AI 自選）、鏡頭沒看到那個洞、
     距離還不穩（1.5 秒內不到六成的距離落在中位數 ±5 cm 內，例如車還在動）、
     飛輪還在加速（實際 Falcon 跟指令差超過 1.5%）、轉速 0、超出 0.5–15 m 等。
   - 同一個洞口、同一個距離（公分）再按一次 S，會取代舊的那筆。
   - 「刪除最後一筆」「全部清除」（按兩下）在面板上；紀錄存在這台電腦的瀏覽器，
     重新整理不會不見。
3. 按「下載 CSV（給 Excel）」，會存成 `下載項目/shot_log_日期_時間.csv`。
4. **先關掉 Excel**，在專案資料夾執行：

   ```bash
   python3 tools/shot_table/apply.py --import --upload
   ```

   - `--import` 沒給檔名時，用「下載項目」裡最新的 `shot_log_*.csv`；也可以指定：
     `--import ~/Downloads/shot_log_20261026_2103.csv`。
   - 先只看會改什麼：`python3 tools/shot_table/apply.py --import --check`。
   - 新的點加在表的最後面；Excel 裡已經有同一個洞口、同一個距離的，改成新的
     仰角和轉速（單位換成 %），一樣的就略過。CSV 和 Excel 都照下面的規則嚴格
     檢查，有任何錯就什麼都不改。
   - 改之前會把原本的 Excel 複製一份到 `tools/shot_table/backup/`（不進 git）。
   - 只改「射擊表」那一頁的儲存格；曲線圖、說明、下拉選單、你自己加的欄位都不動。
   - Excel 開著時（有 `~$shot_table.xlsx`）會拒絕，請先存檔關閉。
5. 打開 Excel 看一下曲線圖，沒問題就把 `shot_table.xlsx` 和 `ShotTableData.h`
   一起 commit。

## 格式（嚴格）

第 1 列是欄位名稱，下面每一列是一個量到的點：

| 欄位 | 內容 | 規則 |
|---|---|---|
| `洞口` | Dashboard 的洞口編號 | 1–9 的整數（1 左下、2 左中、3 左上 … 9 右上） |
| `距離(m)` | 鎖定時 Dashboard 顯示的**相機距離**（不是捲尺） | 0.5–15，最多 2 位小數 |
| `仰角(us)` | Dashboard 的仰角脈寬 | 500–2500 的整數 |
| `轉速` | Falcon 指令，單位看下一欄 | `%`：0–100，最多 1 位小數；`us`：整數脈寬，必須在正轉那一側：`Flywheel::INVERTED = false` 時是 1500–2000、`true` 時是 1000–1500（程式會自己讀目前的設定） |
| `轉速單位` | 下拉選單 | `%` 或 `us`，每一列可以不同 |

- 這五個欄位名稱必須**一字不差**、各出現一次；工作表名稱「射擊表」不能改。
- `us` 就是 Dashboard「Falcon PWM」顯示的脈寬，產生器換算成 %：
  |脈寬 − 1500| ÷ 5，例如 1710 µs → 42%（`INVERTED = false` 時）。填到反轉
  那一側會被擋下來。
- 若插入其他欄位，**新欄位一定要有名稱**；它們不會被讀取。
- 列的順序不重要（程式會依洞口、距離排序）；完全空白的列會略過。
- 同一個洞口的同一個距離不能出現兩次；有填資料的列，四個欄位都要填。
- 不能有合併儲存格或錯誤值（#REF! 等）。
- **只要有一格不符合，就不會產生檔案、也不會燒錄**，畫面會列出每個錯在哪一格。

## Mega 怎麼用這張表

- 每個洞口一張表。兩個距離之間直線內插；比最近的點還近，用最近的點；比最遠的點
  還遠，用最遠的點（`TuningConstants::ShotTable::EXTRAPOLATE_PAST_LAST_ENTRY`）。
- **沒有資料的洞，借用同一排中間洞**：1、7 用 4；2、8 用 5；3、9 用 6。中間洞也
  沒有資料時，自動模式不會對那一排發射。
- 加分小洞（Orin 編號 1–3）沒有表，自動模式不會對它們發射。
- 半自動／全自動時，LT/RT 的轉速微調、十字鍵的仰角微調會加在查表結果上。

## 曲線圖

- 三張圖，分別是中間那一行三個高度：下排（洞口 4，約 120 cm）、中排（洞口 5，
  約 180 cm）、上排（洞口 6，約 240 cm）。橫軸是到目標的距離。
- **轉速**（左軸，紅色）：量測點加上 Excel 的**乘冪趨勢線**（y = a·xᵇ），從原點
  出發、往右上彎、越遠越接近一條斜線。點越多，曲線越準；某個點明顯離開曲線，
  通常就是量歪了。
- **仰角**（右軸，藍色虛線）：量測點依距離連起來。
- 曲線只是用來看資料。**Mega 實際上是在量測點之間直線內插**，不是照趨勢線打。
- 隱藏的工作表「圖表資料」用公式把洞口 4、5、6 的點依距離排序給圖用，不要改它。
  資料放在第 2–500 列，每個洞口最多 60 個點會畫進圖。

## 其他

- `--export`：Excel 不見或壞掉時，從目前的 `ShotTableData.h` 重建一份
  `shot_table.xlsx`（已存在時不會覆蓋）。
