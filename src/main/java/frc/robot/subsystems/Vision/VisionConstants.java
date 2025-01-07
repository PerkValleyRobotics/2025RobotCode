package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = 
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
   // Basic filtering thresholds
   public static double maxAmbiguity = 0.3; 
   public static double maxZError = 0.75;

   // Standard deviation baselines, for 1 meter distance and 1 tag
   // (Adjusted automatically based on distance a # of tags)
   public static double linearStdDevBaseline = 0.02; // Meters
   public static double angularStdDevBaseline = 0.06; // Radians

   // Standard deviation multipliers for each camera 
   // (Adcust to trust some cameras more than others)
   public static double[] cameraStdDevFactors = 
    new double[] {
        1.0 // Camera 0
    };

    // Multipliers to appluy for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotatino data available

}