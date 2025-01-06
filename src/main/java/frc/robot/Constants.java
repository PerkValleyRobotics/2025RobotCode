// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = Robot.isReal() ? Mode.REAL : simMode;

  // swerve module offsets
  public static final double FRONT_LEFT_OFFSET = currentMode == Mode.SIM ? 0 : 299.26764;
  public static final double FRONT_RIGHT_OFFSET = currentMode == Mode.SIM ? 0 : 266.30856;
  public static final double BACK_LEFT_OFFSET = currentMode == Mode.SIM ? 0 : 264.02328;
  public static final double BACK_RIGHT_OFFSET = currentMode == Mode.SIM ? 0 : 152.3146;


  public static enum Mode {
    REAL, 
    SIM, 
    REPLAY
  }
}
