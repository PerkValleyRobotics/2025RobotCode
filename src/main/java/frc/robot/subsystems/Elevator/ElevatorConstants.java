package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.util.Units;       

public class ElevatorConstants {
    public static final int LEFT_SPARKMAX_ID = 5; 
    public static final int RIGHT_SPARKMAX_ID = 6; 
    
    public static final int ELEVATOR_CURRENT_LIMIT = 40;
    public static final double ELEVATOR_MAX_EXTENSION_METERS = 1.75;
    public static final double ELEVATOR_CLOSEDLOOP_TOLERANCE = .025;
    public static final double SPROCKET_RAD_METERS = Units.inchesToMeters(0.75);

    public static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 1.0/5.0;
    public static final double ELEVATOR_VELOCITY_CONVERSION_FACTOR = 1.0/5.0;

    // Pid Constants
    public static final double ELEVATOR_DOWN_P = 0.2;
    public static final double ELEVATOR_DOWN_I = 0;//0.00075;
    public static final double ELEVATOR_DOWN_D = 0.16;
    public static final double ELEVATOR_DOWN_FF = 0.0;

    // Pid Constants
    public static final double ELEVATOR_UP_P = 18;
    public static final double ELEVATOR_UP_I = 0.0;//0.00075;
    public static final double ELEVATOR_UP_D = 6;
    public static final double ELEVATOR_UP_FF = 0.0;

    // public static final double ELEVATOR_MAX_ACCELERATION = 4200;
    // public static final double ELEVATOR_MAX_VELOCITY = 4200;

    // Reef Heights
    public static final double L0_HEIGHT = 0.02;
    public static final double L1_HEIGHT = 0.5;
    public static final double L2_HEIGHT = 1.8238;
    public static final double L3_HEIGHT = 3.145;
    public static final double L4_HEIGHT = 0.0;
}
