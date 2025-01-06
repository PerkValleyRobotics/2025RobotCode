package frc.robot.subsystems.Drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

  // Drive Train constants
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(16.6);
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(23.773);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(23.773);
  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
  public static final Translation2d[] MODULE_TRANSLATIONS =
    new Translation2d[] { new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(- TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(- TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
    };

  // Drive Motor Configuration
  public static final DCMotor DRIVE_GREARBOX = DCMotor.getNEO(1);
  public static double DRIVE_MOTOR_REDUCTION = 6.12;
  public static double DRIVE_CURRENT_LIMIT = 50;

  // Path planner constants
  public static final double ROBOT_MASS = 115;
  public static final double ROBOT_MOI = 0;
  public static final double WHEEL_RADIOUS_METERS = Units.inchesToMeters(4.0) / 2.0;
  public static final double WHEEL_COF = 0;

  public static final RobotConfig ppConfig = 
    new RobotConfig(
      ROBOT_MASS, 
      ROBOT_MOI,
      new ModuleConfig(
        WHEEL_RADIOUS_METERS,
        MAX_LINEAR_SPEED, 
        WHEEL_COF, 
        DRIVE_GREARBOX.withReduction(DRIVE_MOTOR_REDUCTION),
        DRIVE_CURRENT_LIMIT,
        1),
      MODULE_TRANSLATIONS  
    );
}

