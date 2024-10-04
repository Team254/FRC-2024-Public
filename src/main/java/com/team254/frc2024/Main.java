// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.frc2024;

import edu.wpi.first.wpilibj.RobotBase;

import java.util.Arrays;

public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        Constants.kIsReplay = Arrays.asList(args).contains("--replay");
        if (Constants.kIsReplay) {
            System.out.println("Starting Replay mode!");
        }
        RobotBase.startRobot(Robot::new);
    }
}
