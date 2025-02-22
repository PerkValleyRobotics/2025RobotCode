package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.util.Units;       

public class ElevatorConstants {
    public static final int LEFT_SPARKMAX_ID = 5; 
    public static final int RIGHT_SPARKMAX_ID = 6; 
    
    public static final int ELEVATOR_CURRENT_LIMIT = 40;
    public static final double ELEVATOR_MAX_EXTENSION_METERS = 1.75;
    public static final double ELEVATOR_CLOSEDLOOP_TOLERANCE = .05;
    public static final double SPROCKET_RAD_METERS = Units.inchesToMeters(0.75);

    public static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 1.0/5.0;
    public static final double ELEVATOR_VELOCITY_CONVERSION_FACTOR = 1.0/5.0;

    // Pid Constants
    public static final double ELEVATOR_P = .6;
    public static final double ELEVATOR_I = 0.00075;
    public static final double ELEVATOR_D = 0.3;
    public static final double ELEVATOR_FF = 0.0;

    public static final double ELEVATOR_MAX_ACCELERATION = 5;
    public static final double ELEVATOR_MAX_VELOCITY = 5;

    // Reef Heights
    public static final double L0_HEIGHT = 0.02;
    public static final double L1_HEIGHT = 0.5;
    public static final double L2_HEIGHT = 1.8238;
    public static final double L3_HEIGHT = 3.1380;
    public static final double L4_HEIGHT = 0.0;
}
