package frc.robot.subsystems.DeepClimb;

import org.littletonrobotics.junction.AutoLog;

public interface DeepClimbIO {
    @AutoLog
    public static class DeepClimbIOInputs {
        public double positionRads = 0.0;

        public boolean connected = false;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(DeepClimbIOInputs inputs) {}

    public default void setPosition(double position) {}
}


