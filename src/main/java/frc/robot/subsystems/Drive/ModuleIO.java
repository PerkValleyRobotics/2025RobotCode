package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double trunVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
    
        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    // Update the set of loggable inputs 
    public default void updateInputs(ModuleIOInputs inputs) {}

    // Run the drive motor at the specified open loop value.
    public default void setDriveOpenLoop(double output) {}

    // Run the turn motor at the specified open loop value.
    public default void setTurnOpenLoop(double output) {}

    // Run the drive motor at the specified velocity.
    public default void setDriveVelocity(double output) {}

    // Run the drive turn at the specified position.
    public default void setTurnPosition(Rotation2d output) {}
} 
