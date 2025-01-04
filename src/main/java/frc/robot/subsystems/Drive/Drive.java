// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIOInputsAutoLogged;
import edu.wpi.first.wpilibj.DriverStation;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(15);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(24.248);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(24.248);
  private static final double DRIVE_BASE_RADIUS = 
    Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];
  private final SysIdRoutine sysId;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions =
    new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
    };

  private SwerveDrivePoseEstimator poseEstimatior = 
    new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  /** Creates a new Drive. */
  public Drive(
    GyroIO gyroIO,
    ModuleIO flModuleIO,
    ModuleIO frModuleIO,
    ModuleIO blModuleIO,
    ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    
    /*
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            MAX_ANGULAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
        () -> DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    */

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odemetry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Congigure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              for (int i = 0; i < 4; i++) {
                modules[i].runCharacterization(voltage.in(Volts));
              }
            },
            null,
            this));
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (Module module : modules){
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      for (Module module :modules) {
        module.stop();
      }
    }

    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Read whell positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] = 
        new SwerveModulePosition(
          modulePositions[moduleIndex].distanceMeters
            - lastModulePositions[moduleIndex].distanceMeters,
          modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Apply odometry update
    poseEstimatior.update(rawGyroRotation, modulePositions);
  }

  // Run the drive at a desired ChasisSpeeds
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module return the optimized state, useful for logging 
      optimizSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpoinsOptimezed", optimizSetpointStates);
  }

  // Stop the drive
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // Returns a command for a quasistatic test the eh specified direction
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  // Returns a command for a dynamic test the eh specified direction
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  // Returns the module states (turn and angle drive velos) for all of the modules
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }


  // Returns the current odometry pose.
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimatior.getEstimatedPosition();
  }

  // Returns the current odometry rotation
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  // Resets the current odometry pose
  public void setPose(Pose2d pose) {
    poseEstimatior.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  // Add a vision measurement to the pose estimator
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimatior.addVisionMeasurement(visionPose, timestamp);
  }

  // Returns the max linier speed in meter per sec
  public double getMaxLinearSpeedMerterPerSec() {
    return MAX_LINEAR_SPEED;
  }

  // Returns the max angular speed in meter per sec
  public double getMaxAngularSpeedMerterPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  // Returns array of module translations
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(- TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(- TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
    };
  }
}

