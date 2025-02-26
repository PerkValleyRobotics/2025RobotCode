// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralSensor;


import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

/** Add your docs here. */
public class CoralSensorIOReal implements CoralSensorIO {
    private final Rev2mDistanceSensor sensor;

    public CoralSensorIOReal() {
        sensor = new Rev2mDistanceSensor(Port.kMXP);
        sensor.setAutomaticMode(true);
        sensor.setRangeProfile(RangeProfile.kHighAccuracy);
    }

    public void updateInputs(CoralIOInputs inputs) {
        inputs.distance = sensor.getRange();
    }
}
