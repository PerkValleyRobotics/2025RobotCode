package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Module extends SubsystemBase{
    private static final double WHEEL_DIAMETER = Units.inchesToMeters(3.75);
    private static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController driveFeedback;
    private final PIDController turnFeedback;
    private Rotation2d angleSetpoint = null; // Setpoint for closed loop control
    private Double speedSetpoint = null; // Setpoint for closed loop control
    private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute

    // Just testing something here
    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                driveFeedforward = new SimpleMotorFeedforward(0.011306, 0.13225, 0.0041316);
                driveFeedback = new PIDController(0.0, 0.0, 0.0);
                turnFeedback = new PIDController(7.5, 0.0, 0.38);
                
                break;
            case SIM:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
                driveFeedback = new PIDController(0.1, 0.0, 0.0);
                turnFeedback = new PIDController(10.0, 0.0, 0.0);
                break;
            default:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
                driveFeedback = new PIDController(0.0, 0.0, 0.0);
                turnFeedback = new PIDController(0.0, 0.0, 0.0);
                break;
        }

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
        setBrakeMode(true);

        SmartDashboard.putNumber("drive_P", driveFeedback.getP());
        SmartDashboard.putNumber("drive_I" , driveFeedback.getI());
        SmartDashboard.putNumber("drive_D" , driveFeedback.getD());
        SmartDashboard.putNumber("turn_P", turnFeedback.getP());
        SmartDashboard.putNumber("turn_I" , turnFeedback.getI());
        SmartDashboard.putNumber("turn_D" , turnFeedback.getD());
    }

    // Run closed loop turn control
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        if(turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
        }

        if (angleSetpoint != null) {
            io.setTurnVoltage(
                turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

            // Run closed loop drive control 
            // only if closed loop turn control is running 
            if (speedSetpoint != null) {
                // cos scailing of velocity setpoint
                double adjustedSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());

                //run drive controller
                double velocityRadPerSec = adjustedSpeedSetpoint / WHEEL_RADIUS;
                io.setDriveVoltage(
                    driveFeedforward.calculate(velocityRadPerSec)
                        + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
            }
        }

        driveFeedback.setP(SmartDashboard.getNumber("drive_P", 0.0));
        driveFeedback.setI(SmartDashboard.getNumber("drive_I", 0.0));
        driveFeedback.setD(SmartDashboard.getNumber("drive_D", 0.0));
        turnFeedback.setP(SmartDashboard.getNumber("turn_P", 0.0));
        turnFeedback.setI(SmartDashboard.getNumber("turn_I", 0.0));
        turnFeedback.setD(SmartDashboard.getNumber("turn_D", 0.0));
    }

    // Run the module with teh specified setpoints. Return the optimized state
    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        // Optimize the state based on current angle
        // Controllers run in "periodic when the setpoint is not null"
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

        // Update setpoints, controllers run in "periodic"
        angleSetpoint = optimizedState.angle;
        speedSetpoint = optimizedState.speedMetersPerSecond;

        return optimizedState;
    }

    public void runCharacterization(double volts) {
        // Closed loop turn control
        angleSetpoint = new Rotation2d();

        // open loop drive control
        io.setDriveVoltage(volts);
        speedSetpoint= null;
    }

    public void stop() {
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);

        // Disable closed loop control
        angleSetpoint = null;
        speedSetpoint = null;
    }

    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    public Rotation2d getAngle() {
        if (turnRelativeOffset == null) {
            return new Rotation2d();
        } else {
            return inputs.turnPosition.plus(turnRelativeOffset);
        }
    }

    public double getPositionMeters() {
        return inputs.drivePositionRad * WHEEL_RADIUS;
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * WHEEL_RADIUS;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public double GetCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }
}
