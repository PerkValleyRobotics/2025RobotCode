// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralSensor;

import org.littletonrobotics.junction.AutoLog;

public interface CoralSensorIO {
    @AutoLog
    public static class CoralIOInputs {
        public double distance = 0;
    }
    public default void updateInputs(CoralIOInputs inputs) {}
}