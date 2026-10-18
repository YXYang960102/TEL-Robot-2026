# 射擊表（Excel → Mega）

`shot_table.xlsx` 是射擊表的唯一來源。Mega 用的 `src/Constants/ShotTableData.h`
由它自動產生，**不要手改那個 .h**。

## 使用方式

1. 用 Excel 打開 `tools/shot_table/shot_table.xlsx` 的「射擊表」工作表，改數字、
   加列或刪列，存檔（保持 .xlsx）。表格旁邊有兩張「到目標距離」的關係曲線圖
   （仰角、轉速，各洞口一條線），改值、加列、刪列都會自動跟著更新。
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

不需要安裝任何 Python 套件。燒錄要有 PlatformIO（`pio`），找不到時會提示。

## 格式（嚴格）

第 1 列是欄位名稱，下面每一列是一個量到的點：

| 欄位 | 內容 | 規則 |
|---|---|---|
| `洞口` | Dashboard 的洞口編號 | 1–9 的整數（1 左下、2 左中、3 左上 … 9 右上） |
| `距離(m)` | 鎖定時 Dashboard 顯示的**相機距離**（不是捲尺） | 0.5–15，最多 2 位小數 |
| `仰角(us)` | Dashboard 的仰角脈寬 | 500–2500 的整數 |
| `轉速(%)` | Falcon 指令 | 0–100，最多 1 位小數 |

- 這四個欄位名稱必須**一字不差**、各出現一次；工作表名稱「射擊表」不能改。
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

- 隱藏的工作表「圖表資料」用公式把每個洞口的點依距離排序，兩張圖讀它，所以
  不需要手動更新。不要改它。
- 資料放在第 2–500 列；每個洞口最多 60 個點會畫進圖。
- 圖上兩點之間的直線，就是 Mega 實際使用的直線內插。

## 其他

- `--export`：Excel 不見或壞掉時，從目前的 `ShotTableData.h` 重建一份
  `shot_table.xlsx`（已存在時不會覆蓋）。
