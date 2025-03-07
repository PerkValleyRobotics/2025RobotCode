package frc.robot.subsystems.DeAlgifier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeAlgifier extends SubsystemBase {
    private DeAlgifierIO io;
    private final DeAlgifierIOInputsAutoLogged inputs = new DeAlgifierIOInputsAutoLogged();

    private double setpoint;

    public DeAlgifier(DeAlgifierIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("DeAlgifier", inputs);
    }

    // pivot funcitons
    public void gotoPos(double goal) {
        io.setPivotPosition(goal);
        this.setpoint = goal;
    }

    public void incrementSetpoint() {
        setpoint += 0.01;
        gotoPos(setpoint);
    }

    public void decrementSetpoint() {
        setpoint -= 0.01;
        gotoPos(setpoint);
    }

    public double getClosedLoopError() {
        return Math.abs(setpoint - inputs.positionRads);
    }

    @AutoLogOutput(key = "DeAlgifier/Setpoint")
    public double getCurrentSetpoint() {
        return setpoint;
    }

    // wheel functions
    public void runWheel(boolean spinFactor) {
        if (spinFactor)
            io.setWheelSpeed(DeAlgifierConstants.WHEEL_SPEED);
        else
            io.setWheelSpeed(-DeAlgifierConstants.WHEEL_SPEED);
    }

    public void runWheel() {
        runWheel(false);
    }

    public void stopWheel() {
        io.setWheelSpeed(0);
    }

    public void out() {
        gotoPos(-0.24);
    }

    public void in() {
        gotoPos(0);
    }
}
