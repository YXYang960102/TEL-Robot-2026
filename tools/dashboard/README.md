# TEL Robot Dashboard

This dashboard reads Arduino telemetry over USB Serial with the browser Web Serial API.

## Arduino telemetry format

The robot prints one line about every 100 ms:

```text
TEL,ms,tx,ty,distance,target_id,valid,ch0,ch1,ch2,ch3,ch8,shooter_ready,shoot_remaining,warn
```

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
