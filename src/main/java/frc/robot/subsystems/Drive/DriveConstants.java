package frc.robot.subsystems.Drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DriveConstants {
  public static final double  ODOMETRY_FREQUENCY = 100.0; // Hz

  // Drive Train constants
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(16.6);
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(23.773);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(23.773);
  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
  public static final Translation2d[] MODULE_TRANSLATIONS =
    new Translation2d[] { 
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(- TRACK_WIDTH_X / 2.0,-TRACK_WIDTH_Y / 2.0),
      new Translation2d(- TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
    };

  // swerve module offsets
  public static final Rotation2d FRONT_LEFT_ZERO_ROTATION = new Rotation2d(0.0);
  public static final Rotation2d FRONT_RIGHT_ZERO_ROTATION = new Rotation2d(0.0);
  public static final Rotation2d BACK_LEFT_ZERO_ROTATION = new Rotation2d(0.0);
  public static final Rotation2d BACK_RIGHT_ZERO_ROTATION = new Rotation2d(0.0);

  // CAN IDs
  public static final int FRONT_LEFT_DRIVE_ID = 1;
  public static final int FRONT_LEFT_TURN_ID= 2;
  public static final int FRONT_LEFT_CANCODER_ID= 3;

  public static final int FRONT_RIGHT_DRIVE_ID = 11;
  public static final int FRONT_RIGHT_TURN_ID= 12;
  public static final int FRONT_RIGHT_CANCODER_ID= 13;

  public static final int BACK_RIGHT_DRIVE_ID = 21;
  public static final int BACK_RIGHT_TURN_ID = 22;
  public static final int BACK_RIGHT_CANCODER_ID= 23;

  public static final int BACK_LEFT_DRIVE_ID = 31;
  public static final int BACK_LEFT_TURN_ID= 32;
  public static final int BACK_LEFT_CANCODER_ID= 33;

  // Drive Motor Configuration
  public static final DCMotor DRIVE_GEARBOX = DCMotor.getNEO(1);
  public static final double DRIVE_MOTOR_REDUCTION = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0) ;
  public static final int DRIVE_CURRENT_LIMIT = 50;
  public static final boolean DRIVE_MOTOR_INVERTED = true;

  // Drive encoder configuration
  public static final double DRIVE_ENCODER_POSITION_FACTOR = 
    2 * Math.PI / DRIVE_MOTOR_REDUCTION; // Rotor Rotations -> Wheel Radians
  public static final double DRIVE_ENCODER_VELOCITY_FACTOR =
    (2 * Math.PI) / 60.0 / DRIVE_MOTOR_REDUCTION; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID Configuration
  public static final double DRIVE_P = 0.001;
  public static final double DRIVE_D = 0;
  public static final double DRIVE_S = 0;
  public static final double DRIVE_V = 0; 
  public static final double DRIVE_SIM_P = 0.05;
  public static final double DRIVE_SIM_D = 0.0;
  public static final double DRIVE_SIM_S = 0.0;
  public static final double DRIVE_SIM_V = 0.0789; 

  // Turn Motor Configuration
  public static final boolean TURN_INVERTED = true;
  public static final int TURN_CURRENT_LIMIT = 50;
  public static final double TURN_MOTOR_REDUCTION = 150.0 / 7.0 ;
  public static final DCMotor TURN_GEARBOX = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean TURN_ENCODER_INVERTED = true;
  public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI / TURN_MOTOR_REDUCTION; // Rotor Rotation -> Wheel Radians
  public static final double TURN_ENCODER_VELOCITY_FACTOR = 2 * Math.PI / 60.0 / TURN_MOTOR_REDUCTION; // totor RPM - > Wheel Rad/Sec

  // Turn PID Configuration
  public static final double TURN_P = 0.35;
  public static final double TURN_D = 0;
  public static final double TURN_SIM_P = 8.0;
  public static final double TURN_SIM_D = 0.0;
  public static final double TURN_PID_MIN_INPUT = 0; // Radians
  public static final double TURN_PID_MAX_INPUT = 2 * Math.PI; // Radians
  
  // Path planner constants
  public static final double ROBOT_MASS = 115;
  public static final double ROBOT_MOI = 0;
  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
  public static final double WHEEL_COF = 0;
  public static final RobotConfig ppConfig = 
    new RobotConfig(
      ROBOT_MASS, 
      ROBOT_MOI,
      new ModuleConfig(
        WHEEL_RADIUS_METERS,
        MAX_LINEAR_SPEED, 
        WHEEL_COF, 
        DRIVE_GEARBOX.withReduction(DRIVE_MOTOR_REDUCTION),
        DRIVE_CURRENT_LIMIT,
        1),
      MODULE_TRANSLATIONS  
    );
}

