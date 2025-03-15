package frc.robot.subsystems.Intake;

import static frc.robot.subsystems.Intake.IntakeConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private double setpoint;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        io.setPosition(setpoint);
    }

    public void goBack() {
        setpoint = CLIMB_SETPOINT;
    }

    public void goHome() {
        setpoint = HOME_SETPOINT;
    }

    @AutoLogOutput(key = "Intake/Setpoint")
    public double getCurrentSetpoint() {
        return setpoint;
    }
}
