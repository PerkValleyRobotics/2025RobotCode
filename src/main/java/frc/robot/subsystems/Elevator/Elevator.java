// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralSensor.CoralSensor;
import frc.robot.subsystems.DeAlgifier.DeAlgifier;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private CoralSensor coralSensor;
  private DeAlgifier deAlgifier;

  private final Timer coralLostTimer = new Timer(); // Timer to track when coral is lost
  private boolean wasCoralDetected = false;

  private double setpoint;

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io, CoralSensor coralSensor, DeAlgifier deAlgifier) {
    this.io = io;
    this.coralSensor = coralSensor;
    this.deAlgifier = deAlgifier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // if (!coralSensor.isOverrided()) {
    // if (coralSensor.isCoralDetected() || (deAlgifier.getCurrentSetpoint() ==
    // -0.24)) {
    // wasCoralDetected = true;
    // coralLostTimer.stop();
    // coralLostTimer.reset();
    // } else {
    // if (wasCoralDetected) {
    // wasCoralDetected = false;
    // coralLostTimer.restart(); // Start timing when coral is lost
    // }
    // if (coralLostTimer.hasElapsed(1.0)) { // Check if 1 second has passed
    // this.home();
    // }
    // }
    // }
  }

  public void incrementSetpoint() {
    setpoint += .005;
    gotoPos(setpoint);
  }

  public void decrementSetpoint() {
    setpoint -= .005;
    gotoPos(setpoint);
  }

  public void home() {
    setpoint = L0_HEIGHT;
    gotoPos(setpoint);
  }

  public void gotoPos(double goal) {
    if (io.getPosition() < 0.01 && goal == 0) {
      return;
    }
    if (goal <= L0_HEIGHT) {
      goal = L0_HEIGHT;
    }
    if (goal >= 4.3) {
      goal = 4.3;
    }
    io.setPosition(goal);
    this.setpoint = goal;
  }

  public void gotoL1() {
    checkForSensor(() -> {
      setpoint = L1_HEIGHT;
      gotoPos(setpoint);
    });
  }

  public void gotoL2() {
    checkForSensor(() -> {
      setpoint = L2_HEIGHT;
      gotoPos(setpoint);
    });
  }

  public void gotoL3() {
    checkForSensor(() -> {
      setpoint = L3_HEIGHT;
      gotoPos(setpoint);
    });
  }

  public void gotoL4() {
    checkForSensor(() -> {
      setpoint = L4_HEIGHT;
      gotoPos(setpoint);
    });
  }

  public void checkForSensor(Runnable action) {
    System.out.println(deAlgifier.getCurrentSetpoint());
    if (!coralSensor.isOverrided()) {
      if (coralSensor.isCoralDetected()) {
        action.run();
      } else if (deAlgifier.getCurrentSetpoint() < -.1
      ){
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
