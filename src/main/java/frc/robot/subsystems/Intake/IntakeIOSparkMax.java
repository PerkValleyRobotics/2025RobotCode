package frc.robot.subsystems.Intake;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOK;
import static frc.robot.subsystems.Intake.IntakeConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.filter.Debouncer;

public class IntakeIOSparkMax implements IntakeIO {
        private final SparkBase sparkMax;
        private final RelativeEncoder intakeEncoder;
        private final Debouncer connectedDebounce = new Debouncer(0.05);
        private final SparkClosedLoopController insteakController;

        public IntakeIOSparkMax() {
                sparkMax = new SparkMax(IntakeConstants.INTAKE_ID, MotorType.kBrushless);
                intakeEncoder = sparkMax.getEncoder();
                insteakController = sparkMax.getClosedLoopController();

                SparkMaxConfig config = new SparkMaxConfig();
                config
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(INTAKE_CURRENT_LIMIT)
                                .voltageCompensation(12.0)
                                .inverted(false);
                config.encoder
                                .uvwAverageDepth(10)
                                .uvwAverageDepth(2);
                config.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pidf(IntakeConstants.INTAKE_P, IntakeConstants.INTAKE_I, IntakeConstants.INTAKE_D,
                                                IntakeConstants.INTAKE_FF);

                tryUntilOK(sparkMax,
                                5,
                                () -> sparkMax.configure(config, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));
                tryUntilOK(sparkMax, 5, () -> intakeEncoder.setPosition(0.0));

        }

        @Override
        public void updateInputs(IntakeIOInputs inputs) {
                // pivot input updates
                sparkStickyFault = false;
                ifOk(sparkMax, intakeEncoder::getPosition, (value) -> inputs.positionRads = value);

                ifOk(sparkMax,
                                new DoubleSupplier[] { sparkMax::getAppliedOutput,
                                                sparkMax::getBusVoltage },
                                (values) -> inputs.appliedVolts = values[0] * values[1]);
                ifOk(sparkMax, sparkMax::getOutputCurrent,
                                (value) -> inputs.currentAmps = value);
                inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
        }

        public void setPosition(double position) {
                insteakController.setReference(position, ControlType.kPosition);
        }

}
