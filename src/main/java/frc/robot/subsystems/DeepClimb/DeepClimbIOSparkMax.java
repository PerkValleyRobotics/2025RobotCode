package frc.robot.subsystems.DeepClimb;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOK;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;

public class DeepClimbIOSparkMax implements DeepClimbIO {
        private final SparkBase sparkMax;
        private final RelativeEncoder deepClimbEncoder;
        private final Debouncer connectedDebounce = new Debouncer(0.05);
        private final SparkClosedLoopController deepClimbController;

        public DeepClimbIOSparkMax() {
                sparkMax = new SparkMax(DeepClimbConstants.DEEPCLIMB_ID, MotorType.kBrushless);
                deepClimbEncoder = sparkMax.getEncoder();
                deepClimbController = sparkMax.getClosedLoopController();
                SparkMaxConfig config = new SparkMaxConfig();
                config
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(DeepClimbConstants.CURRENT_LIMIT)
                                .voltageCompensation(12.0)
                                .inverted(false);
                config.encoder
                                .uvwAverageDepth(10)
                                .uvwAverageDepth(2);
                config.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(DeepClimbConstants.DEEPCLIMB_P, DeepClimbConstants.DEEPCLIMB_I,
                                                DeepClimbConstants.DEEPCLIMB_D,
                                                DeepClimbConstants.DEEPCLIMB_FF);
                tryUntilOK(sparkMax,
                                5,
                                () -> sparkMax.configure(config, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));
                tryUntilOK(sparkMax, 5, () -> deepClimbEncoder.setPosition(0.0));
        }

        @Override
        public void updateInputs(DeepClimbIOInputs inputs) {
                sparkStickyFault = false;
                ifOk(sparkMax, deepClimbEncoder::getPosition, (value) -> inputs.positionRads = value);
                ifOk(sparkMax,
                                new DoubleSupplier[] { sparkMax::getAppliedOutput,
                                                sparkMax::getBusVoltage },
                                (values) -> inputs.appliedVolts = values[0] * values[1]);
                ifOk(sparkMax, sparkMax::getOutputCurrent,
                                (value) -> inputs.currentAmps = value);
                inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
        }

        public void setPosition(double position) {
                deepClimbController.setReference(position, ControlType.kPosition);
        }
}
