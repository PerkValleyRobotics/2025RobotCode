// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralSensor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSensor extends SubsystemBase {
  private CoralSensorIO io;
  private final CoralSensorIOInputsAutoLogged inputs = new CoralSensorIOInputsAutoLogged();

  private boolean override = false;

  /** Creates a new CoralSensor. */
  public CoralSensor(CoralSensorIO io) {
    this.io = io;
  }

  /**
   * Updates the input values from the sensor and logs them to the DS.
   * <p>
   * This method is called once per scheduler run.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Coral Sensor", inputs);
  }

  public boolean isCoralDetected() {
    return !inputs.state;
  }

  public boolean isOverrided() {
    return override;
  }

  public void overrideSensor() {
    override = true;
  }
}
