package frc.robot.subsystems.Insteak;

import org.littletonrobotics.junction.AutoLog;

public class InsteakIO {
    @AutoLog
    public static class InsteakIOInputs {
        public double positionRads = 0.0;
        
        public boolean connected = false;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(InsteakIOInputs inputs) {}

    public default void setPosition(double position) {}

    // public default void getPosition() {return 0;}
}
