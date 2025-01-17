package frc.robot.subsystems.Elevator;

public class ElevatorConstants {
    public static final int LEFT_SPARKMAX_ID = 3; // to be changed when we know what we want to use
    public static final int RIGHT_SPARKMAX_ID = 4; // to be changed when we know what we want to us
    
    public static final int ELEVATOR_CURRENT_LIMIT = 40;
    public static final double ELEVATOR_MAX_EXTENSION_METERS = 1.75;
    public static final double ELEVATOR_CLOSEDLOOP_TOLERANCE = .05;
    public static final double SPROCKET_RAD_METERS = 0.0;

    // doubled because it is a two stage elevator
    public static final double POSITION_CONVERSION_FACTOR = 2 * (2 * Math.PI * SPROCKET_RAD_METERS); 
    public static final double VELOCITY_CONVERSION_FACTOR = 2 * (1.0/60.0 * 2 * Math.PI * SPROCKET_RAD_METERS);
    // Pid Constants
    public static final double ELEVATOR_P = 0.0;
    public static final double ELEVATOR_D = 0.0;
    public static final double ELEVATOR_FF = 0.0;

    public static final double ELEVATOR_MAX_ACCELERATION = 0.0;
    public static final double ELEVATOR_MAX_VELOCITY= 0.0;

    // Reef Heights
    public static final double L1_HEIGHT = 0.25;
    public static final double L2_HEIGHT = 0.5;
    public static final double L3_HEIGHT = 1.0;
    public static final double L4_HEIGHT = 1.5;
}
