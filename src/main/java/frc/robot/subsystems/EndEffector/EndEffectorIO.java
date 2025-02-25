package frc.robot.subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    
   @AutoLog
   public static class EndEffectorIOInputs {
    public double velocityRadsPerSec = 0.0;
    
    public boolean frontRightConnected = false;
    public double frontRightAppliedVolts = 0.0;
    public double frontRightCurrentAmps = 0.0;

    public boolean frontLeftConnected = false;
    public double frontLeftAppliedVolts = 0.0;
    public double frontLeftCurrentAmps = 0.0;

    public boolean backConnected = false;
    public double backAppliedVolts = 0.0;
    public double backCurrentAmps = 0.0;
   }
   
   // Update the set of Laggable inputs
   public default void updateInputs(EndEffectorIOInputs inputs) {}

   // Set the speed of the front right motor
   public default void setFrontRightSpeed(double speed) {}

   // Set the speed of the front left motor
   public default void setFrontLeftSpeed(double speed) {}

   // Set the speed of the back motor
   public default void setBackSpeed(double speed) {}

}
