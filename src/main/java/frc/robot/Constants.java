// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  // swerve module offsets
  //public static final double FRONT_LEFT_OFFSET = 295.6640625;
  public static final double FRONT_LEFT_OFFSET = 299.26764;
  //public static final double FRONT_RIGHT_OFFSET = 265.861140625;
  public static final double FRONT_RIGHT_OFFSET = 266.30856;
  //public static final double BACK_LEFT_OFFSET = 256.81640625;
  public static final double BACK_LEFT_OFFSET = 264.02328;
  //public static final double BACK_RIGHT_OFFSET = 152.490234375;
  public static final double BACK_RIGHT_OFFSET = 152.3146;

  public static enum Mode {
    REAL, 
    SIM, 
    REPLAY
  }
}
