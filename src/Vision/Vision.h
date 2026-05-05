#pragma once
#include <Arduino.h>

class Vision {
public:
    static void init();
    static void update();

    static double getXPred();

private:
    static void parse(String s);
    static void updatePrediction(double x);

    static String rx;
    static double xPred;
};
