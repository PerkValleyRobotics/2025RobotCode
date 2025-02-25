package frc.robot.subsystems.EndEffector;

import static frc.robot.subsystems.EndEffector.EndEffectorConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOK;

import java.io.ObjectInputFilter.Config;
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

public class EndEffectorIOSparkMax implements EndEffectorIO {
        private final SparkBase frontRightSparkMax;
        private final SparkBase frontLeftSparkMax;
        private final SparkBase backSparkMax;

        private final RelativeEncoder frontRightEncoder;
        private final RelativeEncoder frontLeftEncoder;
        private final RelativeEncoder backEncoder;

        private final Debouncer frontRightConnectedDebounce = new Debouncer(0.05);
        private final Debouncer frontLeftConnectedDebounce = new Debouncer(0.05);
        private final Debouncer backConnectedDebounce = new Debouncer(0.05);

        public EndEffectorIOSparkMax() {
                frontRightSparkMax = new SparkMax(FRONT_RIGHT_SPARKMAX_ID, MotorType.kBrushless);
                frontLeftSparkMax = new SparkMax(FRONT_LEFT_SPARKMAX_ID, MotorType.kBrushless);
                backSparkMax = new SparkMax(BACK_SPARKMAX_ID, MotorType.kBrushless);

                frontRightEncoder = frontRightSparkMax.getEncoder();
                frontLeftEncoder = frontLeftSparkMax.getEncoder();
                backEncoder = backSparkMax.getEncoder();

                SparkMaxConfig config = new SparkMaxConfig();
                config
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(END_EFFECTOR_CURRENT_LIMIT)
                                .voltageCompensation(12.0)
                                .inverted(false);
                config.encoder
                                .uvwMeasurementPeriod(10)
                                .uvwAverageDepth(2);

                tryUntilOK(
                                frontRightSparkMax,
                                5,
                                () -> frontRightSparkMax.configure(config, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));

                tryUntilOK(
                                frontLeftSparkMax,
                                5,
                                () -> frontLeftSparkMax.configure(config, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));

                tryUntilOK(
                                backSparkMax,
                                5,
                                () -> backSparkMax.configure(config, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));
        }

        @Override
        public void updateInputs(EndEffectorIOInputs inputs) {
                // Update front right motor
                ifOk(frontRightSparkMax,
                                new DoubleSupplier[] { frontRightSparkMax::getAppliedOutput,
                                                frontRightSparkMax::getBusVoltage },
                                (values) -> inputs.frontRightAppliedVolts = values[0] * values[1]);
                ifOk(frontRightSparkMax, frontRightSparkMax::getOutputCurrent,
                                (value) -> inputs.frontRightCurrentAmps = value);
                inputs.frontRightConnected = frontRightConnectedDebounce.calculate(!sparkStickyFault);

                // Update front left motor
                ifOk(frontLeftSparkMax,
                                new DoubleSupplier[] { frontLeftSparkMax::getAppliedOutput,
                                                frontLeftSparkMax::getBusVoltage },
                                (values) -> inputs.frontLeftAppliedVolts = values[0] * values[1]);
                ifOk(frontLeftSparkMax, frontLeftSparkMax::getOutputCurrent,
                                (value) -> inputs.frontLeftCurrentAmps = value);
                inputs.frontRightConnected = frontLeftConnectedDebounce.calculate(!sparkStickyFault);

                // Update back motor
                ifOk(backSparkMax,
                                new DoubleSupplier[] { backSparkMax::getAppliedOutput, backSparkMax::getBusVoltage },
                                (values) -> inputs.backAppliedVolts = values[0] * values[1]);
                ifOk(backSparkMax, backSparkMax::getOutputCurrent, (value) -> inputs.backCurrentAmps = value);
                inputs.backConnected = backConnectedDebounce.calculate(!sparkStickyFault);
        }

        @Override
        // Set the speed of the front right motor
        public void setFrontRightSpeed(double speed) {
                frontRightSparkMax.set(speed);
        }

        @Override
        // Set the speed of the front left motor
        public void setFrontLeftSpeed(double speed) {
                frontLeftSparkMax.set(speed);
        }

        @Override
        // Set the speed of the back motor
        public void setBackSpeed(double speed) {
                backSparkMax.set(speed);
        }

}
