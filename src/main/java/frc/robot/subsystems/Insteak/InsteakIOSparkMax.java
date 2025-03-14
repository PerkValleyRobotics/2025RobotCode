// package frc.robot.subsystems.Insteak;

// import static
// frc.robot.subsystems.Insteak.InsteakConstants.INSTEAK_CURRENT_LIMIT;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;

// public class InsteakIOSparkMax implements InsteakIO {
// private final SparkBase sparkMax;
// private final RelativeEncoder encoder;

// private final Debouncer connectedDebounce = new Debouncer(0.05);

// public InsteakIOSparkMax() {
// sparkMax = new SparkMax(, MotorType.kBrushless);
// encoder = sparkMax.getEncoder();

// SparkMaxConfig config = new SpakrMaxConfig();
// config
// .idleMode(IdleMode.kBrake)
// .smartCurrentLimit(INSTEAK_CURRENT_LIMIT)
// .voltageCompensation(12.0)
// .inverted(false);
// config.encoder
// .uvwAverageDepth(10)
// .uvwAverageDepth(2);

// }

// }
