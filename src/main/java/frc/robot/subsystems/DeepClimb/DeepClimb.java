package frc.robot.subsystems.DeepClimb;

import static frc.robot.subsystems.DeepClimb.DeepClimbConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeepClimb extends SubsystemBase {
  private DeepClimbIO io;
  private final DeepClimbIOInputsAutoLogged inputs = new DeepClimbIOInputsAutoLogged();

  private double setpoint;

  public DeepClimb(DeepClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("DeepClimb", inputs);

    io.setPosition(setpoint);
  }

  public void goToHomePos() {
    setpoint = HOME_SETPOINT;
  }

  public void goToClimbPos() {
    setpoint = CLIMB_SETPOINT;
  }

  @AutoLogOutput(key = "Insteak/Setpoint")
  public double getCurrentSetpoint() {
    return setpoint;
  }
}
