package frc.robot.subsystems.Drive;

import static frc.robot.subsystems.Drive.DriveConstants.WHEEL_RADIUS_METERS;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    // Just testing something here
    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
        driveDisconnectedAlert = 
            new Alert("Disconnected drive motor on module ", Integer.toString(index) + ".",
            AlertType.kError);

        turnDisconnectedAlert = 
            new Alert("Disconnected turn motor on module ", Integer.toString(index) + ".",
            AlertType.kError);
    }

    // Run closed loop turn control
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
        
        // Calculate positions for odometry 
        int sampleCount = inputs.odometryTimestamps.length;
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * WHEEL_RADIUS_METERS;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
    }

    // Run the module with teh specified setpoints. Return the optimized state
    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        // Optimize the state based on current angle
        state.optimize(getAngle());
        state.cosineScale(inputs.turnPosition);

        // Apply setpoints
        io.setDriveVelocity(state.speedMetersPerSecond / WHEEL_RADIUS_METERS);
        io.setTurnPosition(state.angle);

        return state;
    }

    // Runs the module with the specified output while controlling to zero degrees.
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new Rotation2d());
    }

    // Disables all output to motors.
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    // Returns the curent angle of the module.
    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    // Returns the current drive poisiton of the module in meters.
    public double getPositionMeters() {
        return inputs.drivePositionRad * WHEEL_RADIUS_METERS;
    }

    // Returns the current drive posision of the module in meters.
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * WHEEL_RADIUS_METERS;
    }
    
    // Returns the currrent module position (turn angle and drive position)
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    // Returns the current module state (turn angle and drive velocity)
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    // Returns the module position received this cycle 
    public SwerveModulePosition[] getOdometryPoositions() {
        return odometryPositions;
    }

    // Return the timestamps of th sammples received this cycle.
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    // Returns the module positions in radians
    public double getWheelRadiousCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    // Return the module velocity in rad/sec
    public double getFFCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }
}
