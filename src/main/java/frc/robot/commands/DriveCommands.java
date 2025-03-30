// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;

import static frc.robot.subsystems.Vision.VisionConstants.robotToCamera;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AprilTagPositions;
import frc.robot.subsystems.Drive.Drive;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/sec

  private static final PIDController autoRotatePID = new PIDController(5, 0, 0.3);
  public static final double CAMERA_TO_ROBOT_CENTER_X = 0.0; // forward x m
  public static final double CAMERA_TO_ROBOT_CENTER_Y = 0.0; // x m out from the left of the robot center

  /** Creates a new DriveCommands. */
  public DriveCommands() {
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

  public static Command DriveAndLookAtReef(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    autoRotatePID.setTolerance(Math.toRadians(2.5));
    autoRotatePID.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
        () -> {
          Translation2d linearVelocity = getLinierVelocityFromJoysticks(
              xSupplier.getAsDouble(), ySupplier.getAsDouble());

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

          autoRotatePID.setSetpoint(targetAngle);

          double omega = 0;
          omega = MathUtil.applyDeadband(
              autoRotatePID.calculate(drive.getRotation().getRadians()),
              DEADBAND);

          // Clamp omega to prevent overcorrection
          omega = MathUtil.clamp(omega, -drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularSpeedRadPerSec());

          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinerSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinerSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static Translation2d getLinierVelocityFromJoysticks(double x, double y) {
    // Apply Deadban
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linierDirection = new Rotation2d(Math.atan2(y, x));

    // Square the linier magnitude for more presice control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linierDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  public static Command FPSDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinierVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Retain the original sign of omega
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds and send command
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinerSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinerSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static Command RelativeDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinierVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Retain the original sign of omega
          omega = Math.copySign(omega * omega, omega);

          drive.runVelocity(
              new ChassisSpeeds(linearVelocity.getX() * drive.getMaxLinerSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinerSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec()));
        },
        drive);
  }

  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
            () -> {
              drive.runCharacterization(0.0);
            },
            drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
            () -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              drive.runCharacterization(voltage);
              velocitySamples.add(drive.getFFCharacterizationVelocity());
              voltageSamples.add(voltage);
            },
            drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                  SmartDashboard.putString("kS: ", formatter.format(kS));
                  SmartDashboard.putString("kV: ", formatter.format(kV));
                }));
  }
}
