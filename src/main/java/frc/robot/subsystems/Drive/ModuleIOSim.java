package frc.robot.subsystems.Drive;

import static frc.robot.subsystems.Drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO{
    private static final double LOOP_PERIOD_SECS = 0.02;
    
    private DCMotorSim driveSim;
    private DCMotorSim turnSim;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(DRIVE_SIM_P, 0, DRIVE_SIM_D);
    private PIDController turnController = new PIDController(TURN_SIM_P, 0, TURN_SIM_D);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim() {
        driveSim = 
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, 0.025, DRIVE_MOTOR_REDUCTION),
                DRIVE_GEARBOX);
        
        turnSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(TURN_GEARBOX, 0.004 , TURN_MOTOR_REDUCTION),
                TURN_GEARBOX);

        // Enable wrappinf for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }


    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if(driveClosedLoop) {
            driveAppliedVolts =
                driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }
        if(turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);

        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePositionRad = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.trunVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        driveFFVolts = DRIVE_SIM_S * Math.signum(velocityRadPerSec) + DRIVE_SIM_V * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }
}
