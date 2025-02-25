package frc.robot.subsystems.Elevator;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOK;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkBase leftSparkMax;    
    private final SparkBase rightSparkMax;    
    private final RelativeEncoder rightEncoder;
    private final AbsoluteEncoder absoluteEncoder;
    private final RelativeEncoder leftEncoder;
    private final SparkClosedLoopController controller;

    private final Debouncer leftConnectedDebounce = new Debouncer(.05);
    private final Debouncer rightConnectedDebounce = new Debouncer(.05);


    public ElevatorIOSparkMax() {
        leftSparkMax = new SparkMax(LEFT_SPARKMAX_ID, MotorType.kBrushless);
        rightSparkMax = new SparkMax(RIGHT_SPARKMAX_ID, MotorType.kBrushless);
        absoluteEncoder = leftSparkMax.getAbsoluteEncoder();
        leftEncoder = leftSparkMax.getEncoder();
        rightEncoder = rightSparkMax.getEncoder();
        controller = leftSparkMax.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ELEVATOR_CURRENT_LIMIT)
            .voltageCompensation(12.0)
            .inverted(false);
        config
            .encoder
            .positionConversionFactor(ELEVATOR_POSITION_CONVERSION_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        config
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ELEVATOR_P, ELEVATOR_I,
                  ELEVATOR_D, 0.0);
        // config
        //     .closedLoop
        //     .maxMotion
        //     .maxVelocity(ELEVATOR_MAX_VELOCITY)
        //     .maxAcceleration(ELEVATOR_MAX_ACCELERATION);

        tryUntilOK(leftSparkMax, 
            5, 
            () ->
                 leftSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOK(leftSparkMax, 5, () -> leftEncoder.setPosition(0.0));

        SparkMaxConfig followerConfig =  new SparkMaxConfig();
        followerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ELEVATOR_CURRENT_LIMIT)
            .voltageCompensation(12.0)
            .follow(LEFT_SPARKMAX_ID);
        
        tryUntilOK(
            rightSparkMax, 
            5, 
            () ->
                 rightSparkMax.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        tryUntilOK(rightSparkMax, 5, () -> rightEncoder.setPosition(0.0));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Update elevator inputs 
        sparkStickyFault = false;
        ifOk(leftSparkMax, leftEncoder::getPosition, (value) -> inputs.positionRads = value);
        ifOk(leftSparkMax, leftEncoder::getVelocity, (value) -> inputs.velocityRadsPerSec = value);

        // Update left motor inputs
        ifOk(leftSparkMax, 
            new DoubleSupplier[] {leftSparkMax::getAppliedOutput, leftSparkMax::getBusVoltage},
            (values) -> inputs.leftAppliedVolts = values[0] * values[1]);
        ifOk(leftSparkMax, leftSparkMax::getOutputCurrent, (value) -> inputs.leftCurrentAmps = value);
        inputs.leftConnected = leftConnectedDebounce.calculate(!sparkStickyFault);

        // Update right motor inputs
        sparkStickyFault = false;
        ifOk(rightSparkMax, 
            new DoubleSupplier[] {rightSparkMax::getAppliedOutput, rightSparkMax::getBusVoltage},
            (values) -> inputs.rightAppliedVolts = values[0] * values[1]);
        ifOk(rightSparkMax, rightSparkMax::getOutputCurrent, (value) -> inputs.rightCurrentAmps = value);
        inputs.rightConnected = rightConnectedDebounce.calculate(!sparkStickyFault);
    }

    // Don't ever use this one if it is connected to the elevator
    @Override
    public void setOpenLoop(double output) {
        leftSparkMax.setVoltage(output);
    }

    @Override
    public void setPosition(double position) {
        controller.setReference(position, SparkBase.ControlType.kPosition);
    }
}
