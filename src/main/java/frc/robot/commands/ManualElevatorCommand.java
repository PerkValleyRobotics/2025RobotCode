// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualElevatorCommand extends Command {
  private final Elevator elevator;
  private final DoubleSupplier heightSupplier;

  public ManualElevatorCommand(Elevator elevator, DoubleSupplier heightsSupplier) {
    this.elevator = elevator;
    this.heightSupplier = heightsSupplier;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (heightSupplier.getAsDouble() >= 0) {
      elevator.gotoPos(heightSupplier.getAsDouble() * ElevatorConstants.ELEVATOR_MAX_EXTENSION_METERS);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.home();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
