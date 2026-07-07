# TEL Robot Dashboard

This dashboard reads Arduino telemetry over USB Serial with the browser Web Serial API.

## Arduino telemetry format

The robot prints one line about every 100 ms:

```text
TEL,ms,tx,ty,distance,target_id,valid,ch0,ch1,ch2,ch3,ch8,shooter_ready,shoot_remaining,warn
```

The dashboard also accepts optional pose fields after `warn`:

```text
TEL,ms,tx,ty,distance,target_id,valid,ch0,ch1,ch2,ch3,ch8,shooter_ready,shoot_remaining,warn,robot_x,robot_y,heading
```

If `robot_x`, `robot_y`, and `heading` are not present, the field view shows that robot position is unknown.

Warn bits:

- `1`: Vision has no valid target.
- `2`: Target distance is below 900, so autonomous forward movement should be blocked.
- `4`: `abs(tx)` is above 120, so autonomous should rotate before moving forward.

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
- Drag the bottom-right corner of a panel to resize it.
- Click `完成布局` to lock the layout.
- Click `重設布局` to return to the default layout.

The layout is saved in browser `localStorage`, so it stays on the same computer/browser after refresh.

## Phone demo

To test the dashboard layout on a phone before Arduino hardware is connected:

1. Start the local server on the Mac.
2. Find the Mac IP address on the same Wi-Fi network.
3. Open `http://MAC_IP:8080` on the phone.
4. Tap `模擬資料`.

The phone demo uses generated telemetry. Real Arduino USB Serial monitoring should be done on the Mac with Chrome or Edge.
