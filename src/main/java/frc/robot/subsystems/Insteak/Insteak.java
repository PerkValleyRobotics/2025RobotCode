// package frc.robot.subsystems.Insteak;

// import static frc.robot.subsystems.Insteak.InsteakConstants.*;

// import org.littletonrobotics.junction.AutoLogOutput;
// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Insteak extends SubsystemBase {
// private InsteakIO io;
// private final InsteakIOInputsAutoLogged inputs = new
// InsteakIOInputsAutoLogged();

// private double setpoint;

// public Insteak(InsteakIO io) {
// this.io = io;
// }

// @Override
// public void periodic() {
// io.updateInputs(inputs);
// Logger.processInputs("EndEffector", inputs);
// }

// public void goBack() {
// setpoint = CLIMB_SETPOINT;
// }

// public void goHome() {
// setpoint = HOME_SETPOINT;
// }

// @AutoLogOutput(key = "Insteak/Setpoint")
// public double getCurrentSetpoint() {
// return setpoint;
// }
// }
