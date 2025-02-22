package frc.robot.subsystems.EndEffector;

import static frc.robot.subsystems.EndEffector.EndEffectorConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
    private EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();    

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
    }

    public void runLeft(boolean spinFactor) {
        if (spinFactor) io.setFrontLeftSpeed(FRONT_LEFT_SPEED/SPIN_FACTOR);
        else io.setFrontLeftSpeed(FRONT_LEFT_SPEED);
    };

    public void runRight(boolean spinFactor) {
        if (spinFactor) io.setFrontRightSpeed(FRONT_RIGHT_SPEED/SPIN_FACTOR);
        else io.setFrontRightSpeed(FRONT_RIGHT_SPEED);
    };

    public void runBack() {
        io.setBackSpeed(BACK_SPEED);
    };

    public void stopFront() {
        io.setFrontLeftSpeed(0);
        io.setFrontRightSpeed(0);
    }

    public void stopBack() {
        io.setBackSpeed(0);
    }
}
