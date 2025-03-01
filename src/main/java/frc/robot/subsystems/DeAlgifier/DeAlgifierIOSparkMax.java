package frc.robot.subsystems.DeAlgifier;

import static frc.robot.subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOK;

import java.util.function.DoubleSupplier;

import static frc.robot.util.SparkUtil.tryUntilOK;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class DeAlgifierIOSparkMax implements DeAlgifierIO {
        private final SparkBase deAlgifierArmPivotSparkMax;
        private final SparkBase deAlgifierWheelSparkMax;
        private final RelativeEncoder wheelEncoder;
        private final RelativeEncoder pivotEncoder;
        private final SparkClosedLoopController pivotController;

        private final Debouncer wheelDebouncer = new Debouncer(0.05);
        private final Debouncer armDebouncer = new Debouncer(0.05);

        public DeAlgifierIOSparkMax() {
                deAlgifierArmPivotSparkMax = new SparkMax(DeAlgifierConstants.ALGAE_PIVOT_ARM_ID, MotorType.kBrushless);
                deAlgifierWheelSparkMax = new SparkMax(DeAlgifierConstants.ALGAE_WHEEL_ID, MotorType.kBrushless);
                pivotEncoder = deAlgifierArmPivotSparkMax.getEncoder();
                wheelEncoder = deAlgifierWheelSparkMax.getEncoder();
                pivotController = deAlgifierArmPivotSparkMax.getClosedLoopController();

                SparkMaxConfig pivotConfig = new SparkMaxConfig();
                pivotConfig
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(DeAlgifierConstants.ALGAE_PIVOT_CURRENT_LIMIT)
                                .voltageCompensation(12)
                                .inverted(false)
                                .softLimit
                                .reverseSoftLimit(0)
                                .forwardSoftLimit(-.4);
                pivotConfig.encoder
                                .positionConversionFactor(DeAlgifierConstants.PIVOT_POSITION_CONVERSION_FACTOR)
                                .uvwMeasurementPeriod(10)
                                .uvwAverageDepth(2);
                pivotConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(DeAlgifierConstants.ALGAE_PIVOT_P, DeAlgifierConstants.ALGAE_PIVOT_I,
                                                DeAlgifierConstants.ALGAE_PIVOT_D, 0.0);

                tryUntilOK(deAlgifierArmPivotSparkMax,
                                5,
                                () -> deAlgifierArmPivotSparkMax.configure(pivotConfig, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));
                tryUntilOK(deAlgifierArmPivotSparkMax, 5, () -> pivotEncoder.setPosition(0.0));

                SparkMaxConfig wheelConfig = new SparkMaxConfig();

                wheelConfig
                                .idleMode(IdleMode.kCoast)
                                .smartCurrentLimit(DeAlgifierConstants.ALGAE_PIVOT_CURRENT_LIMIT) // TODO: fix constant name
                                .voltageCompensation(12.0)
                                .inverted(false);
                wheelConfig.encoder
                                .uvwMeasurementPeriod(10)
                                .uvwAverageDepth(2);

                tryUntilOK(deAlgifierWheelSparkMax,
                                5,
                                () -> deAlgifierWheelSparkMax.configure(wheelConfig, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));
                tryUntilOK(deAlgifierWheelSparkMax, 5, () -> wheelEncoder.setPosition(0.0));
        }

        @Override
        public void updateInputs(DeAlgifierIOInputs inputs) {
                // pivot input updates
                sparkStickyFault = false;
                ifOk(deAlgifierArmPivotSparkMax, pivotEncoder::getPosition, (value) -> inputs.positionRads = value);
                ifOk(deAlgifierArmPivotSparkMax, pivotEncoder::getVelocity,
                                (value) -> inputs.velocityRadsPerSec = value);

                ifOk(deAlgifierArmPivotSparkMax,
                                new DoubleSupplier[] { deAlgifierArmPivotSparkMax::getAppliedOutput,
                                                deAlgifierArmPivotSparkMax::getBusVoltage },
                                (values) -> inputs.ArmAppliedVolts = values[0] * values[1]);
                ifOk(deAlgifierArmPivotSparkMax, deAlgifierArmPivotSparkMax::getOutputCurrent,
                                (value) -> inputs.ArmCurrentAmps = value);
                inputs.ArmConnected = armDebouncer.calculate(!sparkStickyFault);

                // wheel input updates
                ifOk(deAlgifierWheelSparkMax,
                                new DoubleSupplier[] { deAlgifierWheelSparkMax::getAppliedOutput,
                                                deAlgifierWheelSparkMax::getBusVoltage },
                                (values) -> inputs.WheelAppliedVolts = values[0] * values[1]);
                ifOk(deAlgifierWheelSparkMax, deAlgifierWheelSparkMax::getOutputCurrent,
                                (value) -> inputs.WheelCurrentAmps = value);
                inputs.WheelConnected = wheelDebouncer.calculate(!sparkStickyFault);

        }

        @Override
        public void setOpenLoop(double output) {
                deAlgifierArmPivotSparkMax.setVoltage(output);
        }

        @Override
        public void setPivotPosition(double position) {
                pivotController.setReference(position, SparkBase.ControlType.kPosition);
        }

        @Override 
        public void setWheelSpeed(double speed)
        {
                deAlgifierWheelSparkMax.set(speed);
        }

}
