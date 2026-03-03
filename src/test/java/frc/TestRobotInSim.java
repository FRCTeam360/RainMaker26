// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// add to each: (personal note)
//public static void reportError(String error, StackTraceElement[] stackTrace) {
//    reportErrorImpl(true, 1, error, stackTrace);
//  }


package frc;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

public class TestRobotInSim {
    @Test
    static Robot robotTest() {
        try {
            TestRobotInSim.robotTest();
        } catch (Throwable throwable) {
            Throwable cause = throwable.getCause();
            if (cause != null) {
                throwable = cause;
            }
            Robot.reportError(
                "Unhandled exception: " + throwable.toString(), throwable.getStackTrace());
            System.exit(-1);
        }
    }


}
