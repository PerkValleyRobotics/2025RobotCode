package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double [] {};

        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double trunVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double [] {};
    
        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    // Update the set of loggable inputs 
    public default void updateInputs(ModuleIOInputs inputs) {}

    // Run the drive motor at a specified voltage 
    public default void setDriveVoltage(double volts) {}

    // Run the turn motor at a specified voltage
    public default void setTurnVoltage(double volts) {}

    // Enable or disable brake mode ont eh drive motor
    public default void setDriveBrakeMode(boolean enable) {}

    // Enable or disable brake mode on the turn motor
    public default void setTurnBrakeMode(boolean enable) {}
} 
