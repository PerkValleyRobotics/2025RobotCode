package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double velocityRadsPerSec =  0.0;
        public double positionRads = 0.0;

        public boolean leftConnected = false;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;

        public boolean rightConnected = false;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
    }

    // Update the set of laggable inputs
    public default void updateInputs(ElevatorIOInputs inputs) {}

    // Run both motors at a specified open loop value
    public default void setOpenLoop(double output) {}

    // Run the motors to the specified position
    public default void setPosition(double position) {}
}
