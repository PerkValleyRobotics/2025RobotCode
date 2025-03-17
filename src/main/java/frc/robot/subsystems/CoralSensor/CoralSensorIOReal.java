// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralSensor;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class CoralSensorIOReal implements CoralSensorIO {
    private final DigitalInput sensor;
    private final int SENSOR_PORT = 1;

    public CoralSensorIOReal() {
        sensor = new DigitalInput(SENSOR_PORT);
    }

    public void updateInputs(CoralSensorIOInputs inputs) {
        inputs.state = sensor.get();
    }
}
