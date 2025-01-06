// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.Drive.DriveConstants.*;

import java.util.function.DoubleSupplier;
import frc.robot.Constants;
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
          Translation2d linearVelocity =  getLinierVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Retain the original sign of omega
          omega = Math.copySign(omega * omega, omega);


          // Convert to field relative speeds and send command
          boolean isFlipped = 
            DriverStation.getAlliance().isPresent()
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
}
