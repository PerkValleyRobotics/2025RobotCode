package frc.robot.subsystems.Drive;

import static frc.robot.subsystems.Drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.Queue;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class ModuleIOSparkMax implements ModuleIO {
    private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkBase driveSparkMax;
    private final SparkBase turnSparkMax;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final CANcoder cancoder;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Closed Loop controllers 
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    //Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(.05);
    private final Debouncer turnConnectedDebounce = new Debouncer(.05);

    // Status Signals
    private final StatusSignal<Angle> turnAbsolutePosition;

    public ModuleIOSparkMax(int module) {
        zeroRotation = switch (module) {
            case 0 -> FRONT_LEFT_ZERO_ROTATION;
            case 1 -> FRONT_RIGHT_ZERO_ROTATION;
            case 2 -> BACK_RIGHT_ZERO_ROTATION;
            case 3 -> BACK_LEFT_ZERO_ROTATION;
            default -> new Rotation2d();
        };
        driveSparkMax = new SparkMax(
                switch (module) {
                    case 0 -> FRONT_LEFT_DRIVE_ID;
                    case 1 -> FRONT_RIGHT_DRIVE_ID;
                    case 2 -> BACK_RIGHT_DRIVE_ID;
                    case 3 -> BACK_LEFT_DRIVE_ID;
                    default -> 0;
                },
                MotorType.kBrushless);
        turnSparkMax = new SparkMax(
                switch (module) {
                    case 0 -> FRONT_LEFT_TURN_ID;
                    case 1 -> FRONT_RIGHT_TURN_ID;
                    case 2 -> BACK_RIGHT_TURN_ID;
                    case 3 -> BACK_LEFT_TURN_ID;
                    default -> 0;
                },
                MotorType.kBrushless);
        cancoder = new CANcoder(
                switch (module) {
                    case 0 -> FRONT_LEFT_CANCODER_ID;
                    case 1 -> FRONT_RIGHT_CANCODER_ID;
                    case 2 -> BACK_RIGHT_CANCODER_ID;
                    case 3 -> BACK_LEFT_CANCODER_ID;
                    default -> 0;
                });
        driveEncoder = driveSparkMax.getEncoder();
        turnEncoder = turnSparkMax.getEncoder();
        driveController = driveSparkMax.getClosedLoopController();
        turnController = turnSparkMax.getClosedLoopController();

        // CTRE stuff for cancoder
        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnAbsolutePosition.setUpdateFrequency(50.0);
        
        // Drive motor config
        var driveConfig = new SparkMaxConfig();
        driveConfig
            .idleMode(IdleMode.kBrake)
            .inverted(DRIVE_MOTOR_INVERTED)
            .smartCurrentLimit(DRIVE_CURRENT_LIMIT)
            .voltageCompensation(12.0);
        driveConfig
            .encoder
            .positionConversionFactor(DRIVE_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(DRIVE_ENCODER_VELOCITY_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        driveConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                DRIVE_P, 0.0,
                DRIVE_D, 0.0);
        driveConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        tryUntilOK(
            driveSparkMax,
            5,
            () -> 
                driveSparkMax.configure(
                    driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        tryUntilOK(driveSparkMax, 5, () -> driveEncoder.setPosition(0.0));

        // Turn Motor Config 
        var turnConfig = new SparkMaxConfig();
        turnConfig
            .inverted(TURN_INVERTED)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(TURN_CURRENT_LIMIT)
            .voltageCompensation(12.0);
        turnConfig
            .encoder
            .positionConversionFactor(TURN_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(TURN_ENCODER_VELOCITY_FACTOR)
            .uvwAverageDepth(2);
        turnConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(TURN_PID_MIN_INPUT, TURN_PID_MAX_INPUT)
            .pidf(TURN_P, 0.0, TURN_D, 0.0);
        turnConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        
        tryUntilOK(
            turnSparkMax, 
            5, 
            () ->
                turnSparkMax.configure(
                    turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        
        // Seed the turn relatinve encoder from Cancoder Absolute Value.
        turnAbsolutePosition.refresh();
        tryUntilOK(turnSparkMax, 5, () -> turnEncoder.setPosition(turnAbsolutePosition.getValueAsDouble() * (2 * Math.PI)));

        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimeStampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSparkMax, driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSparkMax, turnEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        sparkStickyFault = false;
        ifOk(driveSparkMax, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
        ifOk(driveSparkMax, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
        ifOk(
            driveSparkMax,
            new DoubleSupplier[] {driveSparkMax::getAppliedOutput, driveSparkMax::getBusVoltage},
            (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveSparkMax, driveSparkMax::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        sparkStickyFault = false;
        ifOk(
            turnSparkMax,
            turnEncoder::getPosition, 
            (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
        ifOk(turnSparkMax, turnEncoder::getVelocity, (value) -> inputs.trunVelocityRadPerSec = value);
        ifOk(
            turnSparkMax,
            new DoubleSupplier[] {turnSparkMax::getAppliedOutput, turnSparkMax::getBusVoltage},
            (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSparkMax, turnSparkMax::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);
        
        // Update odometry inputs
        inputs.odometryTimestamps = 
            timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions =
            turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveSparkMax.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSparkMax.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double ffVolts = DRIVE_S * Math.signum(velocityRadPerSec) + DRIVE_V * velocityRadPerSec;
        driveController.setReference(
            velocityRadPerSec,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts,
            ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint =
            MathUtil.inputModulus(
                rotation.plus(zeroRotation).getRadians(), TURN_PID_MIN_INPUT, TURN_PID_MAX_INPUT);
        turnController.setReference(setpoint, ControlType.kPosition);
    }

}