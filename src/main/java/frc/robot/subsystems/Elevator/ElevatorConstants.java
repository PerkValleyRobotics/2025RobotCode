package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.util.Units;       

public class ElevatorConstants {
    public static final int LEFT_SPARKMAX_ID = 5; 
    public static final int RIGHT_SPARKMAX_ID = 6; 
    
    public static final int ELEVATOR_CURRENT_LIMIT = 40;
    public static final double ELEVATOR_MAX_EXTENSION_METERS = 1.75;
    public static final double ELEVATOR_CLOSEDLOOP_TOLERANCE = .05;
    public static final double SPROCKET_RAD_METERS = Units.inchesToMeters(0.75);

    // Pid Constants
    public static final double ELEVATOR_P = .4;
    public static final double ELEVATOR_D = 0.0;
    public static final double ELEVATOR_FF = 0.0;

    public static final double ELEVATOR_MAX_ACCELERATION = 5;
    public static final double ELEVATOR_MAX_VELOCITY = 5;

    // Reef Heights
    public static final double L1_HEIGHT = 0.25;
    public static final double L2_HEIGHT = 0.5;
    public static final double L3_HEIGHT = 1.0;
    public static final double L4_HEIGHT = 1.5;
}
