package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    private LinearSystem<N2, N1, N2> elevatorSystem = 
        LinearSystemId.createElevatorSystem(
            DCMotor.getNEO(2),
            20,
            Units.inchesToMeters(0.5),
            6);
    private ElevatorSim sim; 
    private LoggedMechanism2d mech;
    private LoggedMechanismRoot2d root;
    private LoggedMechanismLigament2d elevator;

    private PIDController controller;
    private double outputVoltage;
    private boolean closedLoop = false;

    public ElevatorIOSim() {
       sim = new ElevatorSim(
                elevatorSystem,
                DCMotor.getNEO(2),
                0,
                2.5,
                true,
                0,
                0, 0);
        
        controller = new PIDController(256, 0.0, 0.0);
        mech = new LoggedMechanism2d(3, 3);
        root = mech.getRoot("elevator", 2, 0);
        elevator = root.append(new LoggedMechanismLigament2d("elevator", 0, 90));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        // run closed loop control 
        if (closedLoop) {
            outputVoltage = controller.calculate(sim.getPositionMeters());
        } else {
            controller.reset();
        }

        sim.setInputVoltage(outputVoltage);
        sim.update(LOOP_PERIOD_SECS);
        
        inputs.leftConnected = true;
        inputs.rightConnected = true;
        inputs.leftCurrentAmps = sim.getCurrentDrawAmps();
        inputs.leftAppliedVolts = outputVoltage;
        inputs.rightAppliedVolts = outputVoltage;
        inputs.rightCurrentAmps = sim.getCurrentDrawAmps();

        inputs.positionRads = sim.getPositionMeters();

        inputs.velocityRadsPerSec = sim.getVelocityMetersPerSecond();

        elevator.setLength(.25 + sim.getPositionMeters());
        Logger.recordOutput("Elevator/Mechanism", mech);
    }

    @Override
    public void setOpenLoop(double output) {
        closedLoop = false;
        outputVoltage = output;
    }

    @Override
    public void setPosition(double setPoint) {
        closedLoop = true;
        controller.setSetpoint(setPoint);
        Logger.recordOutput("Elevator/Setpoint", setPoint);
    }
}
