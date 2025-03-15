// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.AprilTagPositions;
import frc.robot.subsystems.Drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToNearestReefSideCommand extends Command {
  private Command fullPath;
  private Drive drive;
  private boolean isLeftBumper = false;
  private boolean centerAlign = false;

  private final PathConstraints drivetrainConstraints = new PathConstraints(
      2, 1.5,
      Units.degreesToRadians(540), Units.degreesToRadians(720));

  /** Creates a new DriveToNearestReefSideCommand. */
  public DriveToNearestReefSideCommand(Drive drive, boolean isLeftBumper, boolean centerAlign) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.isLeftBumper = isLeftBumper;
    this.centerAlign = centerAlign;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d closestAprilTagPose = getClosestReefAprilTagPose(); // where we want to be at the end

    Command pathfindPath = AutoBuilder.pathfindToPose(
        translateCoord(closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -1.5),
        drivetrainConstraints);
    if (findDistanceBetween(drive.getPose(), closestAprilTagPose) < 0.5) {
      pathfindPath = Commands.defer(() -> generateGoToPath(drive.getPose(),
          translateCoord(drive.getPose(), drive.getPose().getRotation().getDegrees(), -0.5)), Set.of(drive));
    } else if (findDistanceBetween(drive.getPose(), closestAprilTagPose) < 1.5) {
      pathfindPath = new PrintCommand("sigma on the wall");

    }
    try {
      fullPath = pathfindPath.andThen(Commands
          .defer(() -> generateGoToPath(drive.getPose(), closestAprilTagPose), Set.of(drive)));
      fullPath.schedule();
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  private Command generateGoToPath(Pose2d currentPose, Pose2d closestAprilTagPose) {
    PathPlannerPath pathToFront = new PathPlannerPath(
        PathPlannerPath.waypointsFromPoses(
            currentPose,
            closestAprilTagPose),
        drivetrainConstraints,
        null,
        new GoalEndState(0.0, closestAprilTagPose.getRotation()));
    pathToFront.preventFlipping = true;
    return AutoBuilder.followPath(pathToFront);
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
    Integer aprilTagNum = -1;

    for (Map.Entry<Integer, Pose2d> entry : aprilTagsToAlignTo.entrySet()) {
      Pose2d pose = entry.getValue();
      double distance = findDistanceBetween(currentPose, pose);
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = pose;
        aprilTagNum = entry.getKey();
      }
    }

    Pose2d inFrontOfAprilTag = translateCoord(closestPose, closestPose.getRotation().getDegrees(),
        -Units.inchesToMeters(17));

    Pose2d leftOrRightOfAprilTag = inFrontOfAprilTag;
    if (!centerAlign) {
      if (isLeftBumper) {
        leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90,
            0.1540265 - (0.0254 * 1.5));
      } else {
        leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90,
            -0.1540265 - (0.0254 * 2));
      }

      if (List.of(11, 10, 9, 22, 21, 20).contains(aprilTagNum)) {
        if (isLeftBumper) {
          leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90,
              -0.1540265 - (0.0254 * 2));
        } else {
          leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90,
              0.1540265 - (0.0254 * 1.5));
        }
      }
    } else {
      leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, 180, 0.0254);
    }

    return leftOrRightOfAprilTag;
  }

  private Pose2d translateCoord(Pose2d originalPose, double degreesRotate, double distance) {
    double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
    double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

    return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
  }

  private double findDistanceBetween(Pose2d pose1, Pose2d pose2) {
    return Math.sqrt(Math.pow((pose2.getX() - pose1.getX()), 2) + Math.pow((pose2.getY() - pose1.getY()), 2));
  }
}