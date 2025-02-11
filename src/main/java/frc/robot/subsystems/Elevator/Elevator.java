// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double setpoint;
  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void home() {
    io.setPosition(0);
    setpoint = 0;
  }

  public void gotoPos(double setPoint) {
    io.setPosition(setPoint);
    this.setpoint = setPoint;
  }

  public void gotoL1() {
    io.setPosition(L1_HEIGHT);
    setpoint = L1_HEIGHT;
  }

  public void gotoL2() {
    io.setPosition(L2_HEIGHT);
    setpoint = L2_HEIGHT;
  }

  public void gotoL3() {
    io.setPosition(L3_HEIGHT);
    setpoint = L3_HEIGHT;
  }

  public void gotoL4() {
    io.setPosition(L4_HEIGHT);
    setpoint = L4_HEIGHT;
  }

  public double getClosedLoopError() {
    return Math.abs(setpoint - inputs.positionRads);
  }
  
  @AutoLogOutput(key = "Elevator/Setpoint")
  public double getCurrentSetpoint() {
    return setpoint;
  }
}
