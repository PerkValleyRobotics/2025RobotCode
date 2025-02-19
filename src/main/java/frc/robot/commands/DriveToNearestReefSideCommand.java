// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

  private Drive drive;

  /** Creates a new DriveToNearestReefSideCommand. */
  public DriveToNearestReefSideCommand(Drive drive, BooleanSupplier isJoystickMoved) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.isJoystickMoved = isJoystickMoved;
    this.drive = drive;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (isJoystickMoved.getAsBoolean()) {
      ignoreJoystickMovement = true;
    }
    pathfindPath = AutoBuilder.pathfindToPose(
        getClosestReefAprilTagPose(),
        new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)));

    pathfindPath.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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

  private Pose2d getClosestReefAprilTagPose() {
    HashMap<Integer, Pose2d> aprilTagsToAlignTo = AprilTagPositions.WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        aprilTagsToAlignTo = AprilTagPositions.WELDED_RED_CORAL_APRIL_TAG_POSITIONS;
      }
    }

    Pose2d currentPose = drive.getPose();
    Pose2d closestPose = new Pose2d();
    double closestDistance = Double.MAX_VALUE;

    for (Pose2d pose : aprilTagsToAlignTo.values()) {
      double distance = findDistanceBetween(currentPose, pose);
      System.out.println(distance);
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = pose;
      }
    }

    return closestPose;
  }

  private double findDistanceBetween(Pose2d pose1, Pose2d pose2) {
    return Math.sqrt(Math.pow((pose2.getX() - pose1.getX()), 2) + Math.pow((pose2.getY() - pose1.getY()), 2));
  }
}
