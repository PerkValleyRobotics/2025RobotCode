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

  private CoralSensor coralSensor;

  private double setpoint;
  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io, CoralSensor coralSensor) {
    this.io = io;
    this.coralSensor = coralSensor;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (!coralSensor.isOverrided() && !coralSensor.isCoralDetected()) {
        this.home();
    }
  }


  public void incrementSetpoint(){
    setpoint += .005;
    gotoPos(setpoint);
  }
  
  public void decrementSetpoint(){
    setpoint -= .005;
    gotoPos(setpoint);
  }

  public void home() {
    gotoPos(L0_HEIGHT);
  }

  public void gotoPos(double goal) {
    if (io.getPosition() < 0.01 && goal == 0) {
      return;
    }
    if(goal <= L0_HEIGHT) {
      goal = L0_HEIGHT;
    }
    if(goal >= 4.3) {
      goal = 4.3;
    }
    io.setPosition(goal);
    this.setpoint = goal;
  }

  public void gotoL1() {
    checkForSensor(() -> {
      setpoint = L1_HEIGHT;
      gotoPos(setpoint);
    };);
  }

  public void gotoL2() {
    checkForSensor(() -> {
      setpoint = L2_HEIGHT;
      gotoPos(setpoint);
    };);
  }

  public void gotoL3() {
    checkForSensor(() -> {
      setpoint = L3_HEIGHT;
      gotoPos(setpoint);
    };);
  }

  public void gotoL4() {
    checkForSensor(() -> {
      setpoint = L4_HEIGHT;
      gotoPos(setpoint);
    };);
  }

  public void checkForSensor(Runnable action) {
    if (!coralSensor.isOverrided()) {
      if (coralSensor.isCoralDetected()) {
        action.run();
      }
    } else {
      action.run();
    }
  }

  public double getClosedLoopError() {
    return Math.abs(setpoint - inputs.positionRads);
  }
  
  @AutoLogOutput(key = "Elevator/Setpoint")
  public double getCurrentSetpoint() {
    return setpoint;
  }
}
