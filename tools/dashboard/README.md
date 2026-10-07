# TEL Robot Dashboard

This dashboard reads Arduino telemetry over USB Serial with the browser Web Serial API.

## Arduino telemetry format

The robot prints one line about every 100 ms:

```text
TEL,ms,tx,ty,distance,target_id,valid,ch0,ch1,ch2,ch3,ch8,shooter_ready,shoot_remaining,warn
```

The dashboard also accepts two independent sets of optional trailing fields: pose
(`robot_x,robot_y,heading`) and operator/fire state
(`operator_mode,fire_state`). Each is present or absent on its own, so the
total field count (including the leading `TEL` tag) tells
`parseTelemetry()` which combination a line carries — these four counts are
the only valid ones:

```text
15  TEL,ms,...,warn                                            (base only)
17  TEL,ms,...,warn,operator_mode,fire_state                   (+ mode, no pose)
18  TEL,ms,...,warn,robot_x,robot_y,heading                    (+ pose, no mode)
20  TEL,ms,...,warn,robot_x,robot_y,heading,operator_mode,fire_state  (+ both)
```

If `robot_x`/`robot_y`/`heading` are not present, the Field Overview panel
shows that robot position is unknown. If `operator_mode`/`fire_state` are
not present, the State Transitions panel shows both rows in a muted
"not sent by this branch" color.

`operator_mode` values: `0` = FULL_AUTO, `1` = SEMI_AUTO, `2` = FULL_MANUAL.
`fire_state` values: `0` = IDLE, `1` = SPINNING_UP, `2` = READY_TO_FIRE,
`3` = FEEDING, `4` = COOLDOWN (the Shooter branch's `Auto::FireState`,
declaration order).

Warn bits:

- `1`: Vision has no valid target.
- `2`: Target distance is below 900, so autonomous forward movement should be blocked.
- `4`: `abs(tx)` is above 120, so autonomous should rotate before moving forward.

## Panels

Beyond the original metric/log/field panels, three Foxglove-Studio-inspired
panels read the same parsed telemetry object with no extra wiring:

- **Plot** — a scrolling line chart (last 180 packets) of `tx`/`ty`/`distance`,
  with a checkbox per series to show/hide it. Same canvas-redraw approach as
  the existing Realtime Trace panel, just generalized to a configurable
  series list instead of two hardcoded ones.
- **State Transitions** — two color-banded timeline rows (Operator Mode,
  Fire State) over the same packet history, with a text legend. Falls back
  to a muted color and a caption when a branch doesn't send
  `operator_mode`/`fire_state` at all.
- **Raw Messages** — the latest raw `TEL,...` line plus every parsed field
  name/value, generated generically from the parsed object so a future field
  addition shows up here with no panel code changes.

## Use

Serve this folder locally, then open it in Chrome or Edge:

```sh
cd tools/dashboard
python3 -m http.server 8080
```

Open:

```text
http://localhost:8080
```

Click `連接 Arduino`, choose the Arduino Mega USB serial port, and keep the baud rate at `115200`.

Jetson vision data should stay on Arduino `Serial1`; this dashboard uses Arduino USB `Serial`.

## Layout editing

Click `編輯布局` to arrange the dashboard in the browser.

- Drag a panel title bar to move it.
- Drag any panel edge or corner to resize it.
- Click `完成布局` to lock the layout.
- Click `重設布局` to return to the default layout.

The layout is saved in browser `localStorage`, so it stays on the same computer/browser after refresh.

## Vision stream

The `Vision Stream / 視覺串流` panel can display an HTTP image stream from the Jetson.

Recommended Jetson stream format:

```text
MJPEG over HTTP
```

Example URLs:

```text
http://JETSON_IP:5000/video_feed
http://JETSON_IP:8080/stream.mjpg
```

Enter the URL in the stream panel and click `套用串流`. The URL is saved in browser `localStorage`.

Keep telemetry and video separate:

- Jetson to Arduino data: Arduino `Serial1`
- Arduino to dashboard telemetry: Arduino USB `Serial`
- Jetson video to dashboard: HTTP/MJPEG URL

On branches with the ELRS bridge (see `tools/stm32_elrs_robot_bridge`/
`tools/stm32_elrs_ground_bridge`), the Mega also sends this exact same
`TEL,...` line over its mechanism-link serial port; the ground-station
STM32 reassembles it and writes it out a USB-UART adapter, so this
dashboard can read it there too without any code changes — it is still
just a line of text on a serial port.

## Phone demo

To test the dashboard layout on a phone before Arduino hardware is connected:

1. Start the local server on the Mac.
2. Find the Mac IP address on the same Wi-Fi network.
3. Open `http://MAC_IP:8080` on the phone.
4. Tap `模擬資料`.

The phone demo uses generated telemetry. Real Arduino USB Serial monitoring should be done on the Mac with Chrome or Edge.
