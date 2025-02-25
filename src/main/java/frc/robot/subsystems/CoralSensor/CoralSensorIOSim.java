// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralSensor;

import static frc.robot.util.SparkUtil.ifOk;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

/** Add your docs here. */
public class CoralSensorIOSim implements CoralSensorIO {

    public CoralSensorIOSim() {
    }

    public void updateInputs(CoralIOInputs inputs) {
        inputs.distance = 0;
    }
}
