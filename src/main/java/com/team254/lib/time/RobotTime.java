package com.team254.lib.time;

import org.littletonrobotics.junction.Logger;

public class RobotTime {
    public static double getTimestampSeconds() {
        long micros = Logger.getTimestamp();
        return (double) micros * 1.0E-6;
    }
}
