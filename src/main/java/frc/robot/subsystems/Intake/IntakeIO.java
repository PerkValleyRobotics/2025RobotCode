package frc.robot.subsystems.Intake;

// import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double positionRads = 0.0;

        public boolean connected = false;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

//     public default void setPosition(double position) {}

//     // public default void getPosition() {return 0;}
// }
