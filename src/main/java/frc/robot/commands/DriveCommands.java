// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive.Drive;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  /** Creates a new DriveCommands. */
  public DriveCommands() {}

  public static Command joyStickDrive(
    Drive drive,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier omegaSupplier) {
     return Commands.run(
      () -> {
          // Apply deadband
          double liearMagnitude = 
            MathUtil.applyDeadband(
              Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection = 
            new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Suare valuse to get a quadratic relationship between joystick position and robot speed 
          // Gives fine presision for small joystick movements and high speeds for large joystick movements
          liearMagnitude = liearMagnitude * liearMagnitude;
          // Retain the original sign of omega
          omega = Math.copySign(omega * omega, omega);

          // Calculate new linear velocity
          Translation2d linearVelocity = 
            new Pose2d(new Translation2d(), linearDirection)
              .transformBy(new Transform2d(liearMagnitude, 0.0, new Rotation2d()))
              .getTranslation();

          // Convert to field relative speeds and send command
          boolean isFlipped = 
            DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMerterPerSec(), 
              linearVelocity.getY() * drive.getMaxLinearSpeedMerterPerSec(),
              omega * drive.getMaxAngularSpeedMerterPerSec(),
              isFlipped
                ? drive.getRotation().plus(new Rotation2d(Math.PI)) 
                : drive.getRotation()));
      },
      drive);
    }
}
