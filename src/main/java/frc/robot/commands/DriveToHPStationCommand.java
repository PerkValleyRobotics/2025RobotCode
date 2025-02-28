// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AprilTagPositions;
import frc.robot.subsystems.Drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToHPStationCommand extends Command {
  private Command fullPath;
  private Drive drive;

  /** Creates a new DriveToHPStationCommand. */
  public DriveToHPStationCommand(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d closestHPStationAprilTagPose = getClosestHPStationAprilTagPose();

    Pose2d inFrontAwayPose = rotatePose(
        translateCoord(closestHPStationAprilTagPose, closestHPStationAprilTagPose.getRotation().getDegrees(), -0.5),
        180);
    Pose2d inFrontFullPose = rotatePose(
        closestHPStationAprilTagPose,
        180);

    Command pathfindPath = AutoBuilder.pathfindToPose(
        inFrontAwayPose,
        new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)));

    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath pathToFront = new PathPlannerPath(
          PathPlannerPath.waypointsFromPoses(
              inFrontAwayPose,
              inFrontFullPose),
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
          null,
          new GoalEndState(0.0, inFrontFullPose.getRotation()));
      pathToFront.preventFlipping = true;
      fullPath = pathfindPath.andThen(AutoBuilder.followPath(pathToFront));
      fullPath.schedule();
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (fullPath != null) {
      fullPath.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private Pose2d getClosestHPStationAprilTagPose() {
    HashMap<Integer, Pose2d> aprilTagsToAlignTo = AprilTagPositions.WELDED_BLUE_HP_STATION_APRIL_TAG_POSITIONS;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        aprilTagsToAlignTo = AprilTagPositions.WELDED_RED_HP_STATION_APRIL_TAG_POSITIONS;
      }
    }

    Pose2d currentPose = drive.getPose();
    Pose2d closestPose = new Pose2d();
    double closestDistance = Double.MAX_VALUE;

    for (Map.Entry<Integer, Pose2d> entry : aprilTagsToAlignTo.entrySet()) {
      Pose2d pose = entry.getValue();
      double distance = findDistanceBetween(currentPose, pose);
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = pose;
      }
    }

    Pose2d inFrontOfAprilTag = translateCoord(closestPose, closestPose.getRotation().getDegrees(),
        -Units.inchesToMeters(23.773));

    return inFrontOfAprilTag;
  }

  private Pose2d translateCoord(Pose2d originalPose, double degreesRotate, double distance) {
    double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
    double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

    return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
  }

  private double findDistanceBetween(Pose2d pose1, Pose2d pose2) {
    return Math.sqrt(Math.pow((pose2.getX() - pose1.getX()), 2) + Math.pow((pose2.getY() - pose1.getY()), 2));
  }

  private Pose2d rotatePose(Pose2d originalPose, double deg) {
    return new Pose2d(originalPose.getX(), originalPose.getY(),
        new Rotation2d(Units.degreesToRadians(originalPose.getRotation().getDegrees() - deg)));
  }
}