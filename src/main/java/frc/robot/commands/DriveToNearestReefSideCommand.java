// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTagPositions;
import frc.robot.subsystems.Drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToNearestReefSideCommand extends Command {
  private Command pathfindPath;
  private Boolean ignoreJoystickMovement = false;
  private BooleanSupplier isJoystickMoved = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return false;
    }
  };

  /** Creates a new DriveToNearestReefSideCommand. */
  public DriveToNearestReefSideCommand(BooleanSupplier isJoystickMoved) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.isJoystickMoved = isJoystickMoved;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (isJoystickMoved.getAsBoolean()) {
      ignoreJoystickMovement = true;
    }
    pathfindPath = AutoBuilder.pathfindToPose(
        AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(6),
        new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)));

    pathfindPath.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (ignoreJoystickMovement && !isJoystickMoved.getAsBoolean()) {
    // ignoreJoystickMovement = false;
    // } else
    if (ignoreJoystickMovement && !isJoystickMoved.getAsBoolean()) {
      ignoreJoystickMovement = false;
    }
    if (!ignoreJoystickMovement && isJoystickMoved.getAsBoolean()) {
      this.cancel();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (pathfindPath != null) {
      pathfindPath.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
