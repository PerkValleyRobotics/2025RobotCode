package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class ModuleIOSparkMax implements ModuleIO {
    private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    private final SparkMax driveSparkMax;
    private final SparkMax turnSparkMax;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnRelativeEncoder;
    private final CANcoder cancoder;

    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;

    private final StatusSignal<Double> turnAbsolutePosition;

    public ModuleIOSparkMax(int index) {
        switch(index) {
            case 0:
                // Front Left
                driveSparkMax = new SparkMax(21, MotorType.kBrushless);
                turnSparkMax = new SparkMax(22, MotorType.kBrushless);
                cancoder = new CANcoder(2);
                absoluteEncoderOffset = Rotation2d.fromDegrees(Constants.FRONT_LEFT_OFFSET);
                break;
            case 1:
                // Front Right
                driveSparkMax = new SparkMax(31, MotorType.kBrushless);
                turnSparkMax = new SparkMax(32, MotorType.kBrushless);
                cancoder = new CANcoder(3);
                absoluteEncoderOffset = Rotation2d.fromDegrees(Constants.FRONT_RIGHT_OFFSET);
                break;
            case 2:
                // Back Left
                driveSparkMax = new SparkMax(11, MotorType.kBrushless);
                turnSparkMax = new SparkMax(12, MotorType.kBrushless);
                cancoder = new CANcoder(1);
                absoluteEncoderOffset = Rotation2d.fromDegrees(Constants.BACK_LEFT_OFFSET);
                break;
            case 3:
                // Back Right
                driveSparkMax = new SparkMax(38, MotorType.kBrushless);
                turnSparkMax = new SparkMax(39, MotorType.kBrushless);
                cancoder = new CANcoder(4);
                absoluteEncoderOffset = Rotation2d.fromDegrees(Constants.BACK_RIGHT_OFFSET);
                break;
            default:
                throw new RuntimeException("Invalid Module Index");
        }

        // driveSparkMax.restoreFactoryDefaults();
        // turnSparkMax.restoreFactoryDefaults();
        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        driveEncoder = driveSparkMax.getEncoder();
        turnRelativeEncoder = turnSparkMax.getEncoder();

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig turnConfig = new SparkMaxConfig();

        driveConfig
            .smartCurrentLimit(40)
            .voltageCompensation(12.0);
        driveConfig.encoder
            .quadratureMeasurementPeriod(10)
            .quadratureAverageDepth(2);
    
        turnConfig
            .smartCurrentLimit(30)
            .voltageCompensation(12.0)
            .inverted(isTurnMotorInverted);
        turnConfig.encoder
            .quadratureMeasurementPeriod(10)
            .quadratureAverageDepth(2);

        driveEncoder.setPosition(0.0);
        turnRelativeEncoder.setPosition(0.0);

        driveSparkMax.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);

        driveSparkMax.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // CTRE stuff for cancoder
        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnAbsolutePosition.setUpdateFrequency(50.0);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = 
            Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec = 
            Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
        inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

        turnAbsolutePosition.refresh();
        inputs.turnAbsolutePosition = 
            Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
        inputs.turnPosition =
            Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
        inputs.trunVelocityRadPerSec = 
            Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
                / TURN_GEAR_RATIO;
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSparkMax.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }

    @Override
    public void setDriveBrakeMode(boolean enabled) {
        driveSparkMax.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setTurnBrakeMode(boolean enabled) {
        turnSparkMax.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    }
}