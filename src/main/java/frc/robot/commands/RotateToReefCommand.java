// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTagPositions;
import frc.robot.subsystems.Drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToReefCommand extends Command {
  private Drive drive;

  private static final PIDController autoRotatePID = new PIDController(5, 0, 0.3);
  public static final double CAMERA_TO_ROBOT_CENTER_X = 0.0; // forward x m
  public static final double CAMERA_TO_ROBOT_CENTER_Y = 0.0; // x m out from the left of the robot center

  private boolean doneRotating = false;

  /** Creates a new RotateToReefCommand. */
  public RotateToReefCommand(Drive drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoRotatePID.setTolerance(Math.toRadians(2.5));
    autoRotatePID.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();

    double cameraOffsetX = CAMERA_TO_ROBOT_CENTER_X *
        Math.cos(robotPose.getRotation().getRadians())
        - CAMERA_TO_ROBOT_CENTER_Y * Math.sin(robotPose.getRotation().getRadians());
    double cameraOffsetY = CAMERA_TO_ROBOT_CENTER_X *
        Math.sin(robotPose.getRotation().getRadians())
        + CAMERA_TO_ROBOT_CENTER_Y * Math.cos(robotPose.getRotation().getRadians());

    Pose2d cameraFieldPose = new Pose2d(
        cameraOffsetX + robotPose.getX(),
        cameraOffsetY + robotPose.getY(),
        robotPose.getRotation());

    Pose2d nearestReefSide = getClosestReefAprilTagPose(drive);

    double targetAngle;
    try {
      targetAngle = Math.atan2(
          nearestReefSide.getY() - cameraFieldPose.getY(),
          nearestReefSide.getX() - cameraFieldPose.getX());
    } catch (Exception e) {
      targetAngle = 0;
    }

    if (Math.abs(drive.getRotation().getDegrees() - Math.toDegrees(targetAngle)) <= Math.toRadians(10)) {
      doneRotating = true;
      // System.out.println(
      // "yiuyoiulyyoyiuyoiulyyoyiuyoiulyyoyiuyoiulyyoyiuyoiulyyoyiuyoiulyyoyiuyoiulyyoyiuyoiulyyoyiuyoiulyyo");
      // this.cancel();
      // System.out.println("overhereoverhereoverhereoverhereoverhereoverhereoverhere");
      // return;
    }

    autoRotatePID.setSetpoint(targetAngle);

    double omega = 0;
    omega = MathUtil.applyDeadband(
        autoRotatePID.calculate(drive.getRotation().getRadians()),
        0.1);

    // Clamp omega to prevent overcorrection
    omega = MathUtil.clamp(omega, -drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularSpeedRadPerSec());

    boolean isFlipped = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            0, 0,
            omega * drive.getMaxAngularSpeedRadPerSec(),
            isFlipped
                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return doneRotating;
  }

  private static double findDistanceBetween(Pose2d pose1, Pose2d pose2) {
    return Math.sqrt(Math.pow((pose2.getX() - pose1.getX()), 2) + Math.pow((pose2.getY() - pose1.getY()), 2));
  }

  private static Pose2d getClosestReefAprilTagPose(Drive drive) {
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

    for (Map.Entry<Integer, Pose2d> entry : aprilTagsToAlignTo.entrySet()) {
      Pose2d pose = entry.getValue();
      double distance = findDistanceBetween(currentPose, pose);
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = pose;
      }
    }

    return closestPose;
  }
}