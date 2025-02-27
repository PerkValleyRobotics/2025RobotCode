package frc.robot.subsystems.DeAlgifier;

import org.littletonrobotics.junction.AutoLog;

public interface DeAlgifierIO {
    @AutoLog
    public static class DeAlgifierIOInputs {
        public double velocityRadsPerSec =  0.0;
        public double positionRads = 0.0;

        public boolean ArmConnected = false;
        public double ArmAppliedVolts = 0.0;
        public double ArmCurrentAmps = 0.0;

        public boolean WheelConnected = false;
        public double WheelAppliedVolts = 0.0;
        public double WheelCurrentAmps = 0.0;
    }
    public default void updateInputs(DeAlgifierIOInputs inputs){}
    
    public default void setOpenLoop(double output) {}

    public default void setPivotPosition(double position) {}

    public default void setWheelSpeed(double speed) {}

    public default void setdealgeasped(double speed) {}
}
