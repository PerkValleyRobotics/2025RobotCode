package frc.robot.subsystems.CoralSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class CoralSensorIOReal implements CoralSensorIO {
    private final DigitalInput sensor;
    private final int SENSOR_PORT = 1;
    private boolean lastValidState;
    private double lastStateChangeTime;
    private static final double STABLE_DURATION = 0.15; // 100ms debounce time

    public CoralSensorIOReal() {
        sensor = new DigitalInput(SENSOR_PORT);
        lastValidState = sensor.get();
        lastStateChangeTime = Timer.getFPGATimestamp();
    }

    public void updateInputs(CoralSensorIOInputs inputs) {
        // boolean currentState = sensor.get();
        // double currentTime = Timer.getFPGATimestamp();

        // if (currentState != lastValidState) {
        //     lastStateChangeTime = currentTime; // Reset timer when state changes
        // }

        // if ((currentTime - lastStateChangeTime) >= STABLE_DURATION) {
        //     lastValidState = currentState; // Accept the new state after it remains stable
        // }

        // inputs.state = lastValidState;
        inputs.state = sensor.get();
    }
}
