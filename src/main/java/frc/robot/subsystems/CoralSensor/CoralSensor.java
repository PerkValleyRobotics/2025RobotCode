// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// //package frc.robot.subsystems.CoralSensor;

// import org.littletonrobotics.junction.AutoLogOutput;
// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class CoralSensor extends SubsystemBase {
//   private CoralSensorIO io;
//   private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();
//   /** Creates a new CoralSensor. */
//   public CoralSensor(CoralSensorIO io) {
//     this.io = io;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     io.updateInputs(inputs);
//     Logger.processInputs("Coral Sensor", inputs);
//   }

//   public double getDistance() {
//     return inputs.distance;
//   }
//   @AutoLogOutput (key="Coral Sensor/isCoralIntaked")
//   public boolean isCoralIntaked() {
//     return inputs.distance < 2;
//   }
// }
