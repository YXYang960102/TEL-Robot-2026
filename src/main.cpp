#include <Arduino.h>
#include "Constants/RobotConstants.h"
#include "Chassis/Chassis.h"
#include "Vision/Vision.h"
#include "Shooter/Shooter.h"
#include "Dribbler/Dribbler.h"

// 建立實例
Chassis chassis;
Vision jetson;
Shooter shooter;
Dribbler dribbler;

// Xbox 數據變數
int xboxX = 1500, xboxY = 1500, xboxR = 1500;
int btnA = 0, btnB = 0, btnMode = 0;

void parseXbox(String input);

void setup() {
    Serial.begin(115200);   // controller
    Serial1.begin(115200);  // for jetson orin

    chassis.begin(
        RobotConfig::Chassis::PIN_FR, RobotConfig::Chassis::PIN_BR,
        RobotConfig::Chassis::PIN_FL, RobotConfig::Chassis::PIN_BL
    );
    shooter.begin();
    dribbler.begin();
}

void loop() {
    // 1. 持續更新視覺預測
    jetson.update();

    // 2. 讀取電腦傳來的 Xbox 數據 (Serial)
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        parseXbox(input);
    }

    // 3. 執行邏輯
    // 模式切換：btnMode == 2 (全自動), 1 (半自動瞄準), 0 (全手動)
    if (btnMode == 2) {
        chassis.stop();
        shooter.update(jetson.getXPred(), jetson.getYRange(), true);
    } 
    else if (btnMode == 1) {
        chassis.drive(xboxX, xboxY, xboxR);
        shooter.update(jetson.getXPred(), jetson.getYRange(), true);
    } 
    else {
        chassis.drive(xboxX, xboxY, xboxR);
        shooter.update(0, 0, false); // 關閉自動瞄準
    }

    // 4. 運球與發射
    dribbler.update(btnA, btnB); // A吸球, B吐球
}

// 解析 Xbox 字串邏輯 (範例格式: X,Y,R,A,B,Mode)
void parseXbox(String input) {
    int comma1 = input.indexOf(',');
    int comma2 = input.indexOf(',', comma1 + 1);
    int comma3 = input.indexOf(',', comma2 + 1);
    int comma4 = input.indexOf(',', comma3 + 1);
    int comma5 = input.indexOf(',', comma4 + 1);

    if (comma1 != -1 && comma5 != -1) {
        int rawX = input.substring(0, comma1).toInt();
        int rawY = input.substring(comma1 + 1, comma2).toInt();
        int rawR = input.substring(comma2 + 1, comma3).toInt();
        
        int dz = RobotConfig::Chassis::DEADZONE;
        int stop = RobotConfig::Chassis::PWM_STOP;

        xboxX = (abs(rawX - stop) < dz) ? stop : rawX;
        xboxY = (abs(rawY - stop) < dz) ? stop : rawY;
        xboxR = (abs(rawR - stop) < dz) ? stop : rawR;

        btnA = input.substring(comma3 + 1, comma4).toInt();
        btnB = input.substring(comma4 + 1, comma5).toInt();
        btnMode = input.substring(comma5 + 1).toInt();
    }
}