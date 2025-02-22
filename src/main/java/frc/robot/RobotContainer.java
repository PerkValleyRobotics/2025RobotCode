// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.Vision.VisionConstants.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToNearestReefSideCommand;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.ModuleIO;
import frc.robot.subsystems.Drive.ModuleIOSim;
import frc.robot.subsystems.Drive.ModuleIOSparkMax;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIONavx;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOLimelight;
import frc.robot.subsystems.Vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;

  private final CommandXboxController driverController = new CommandXboxController(0);

  private final Trigger joystickMoveTrigger = new Trigger(() -> isJoystickMoved());
  // private final CommandXboxController operatorController = new
  // CommandXboxController(1);

  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive = new Drive(
            new GyroIONavx(),
            new ModuleIOSparkMax(0),
            new ModuleIOSparkMax(1),
            new ModuleIOSparkMax(2),
            new ModuleIOSparkMax(3));
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOLimelight("limelight", drive::getRotation));
        elevator = new Elevator(new ElevatorIOSim());

        break;

      case SIM:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim("limelight", robotToCamera, drive::getPose));
        elevator = new Elevator(
            new ElevatorIOSim());
        break;

      default:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
        });
        elevator = new Elevator(new ElevatorIO() {
        });
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    drive.setDefaultCommand(
        DriveCommands.FPSDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Command pathfindPath = AutoBuilder.pathfindToPose(
    // AprilTagPositions.WELDED_APRIL_TAG_POSITIONS.get(6),
    // new PathConstraints(
    // 3.0, 4.0,
    // Units.degreesToRadians(540), Units.degreesToRadians(720)));

    Command driveToNearestReefSideCommandLeft = new DriveToNearestReefSideCommand(drive,
        true);
    Command driveToNearestReefSideCommandRight = new DriveToNearestReefSideCommand(drive,
        false);
    driverController.leftBumper().onTrue(driveToNearestReefSideCommandLeft);

    driverController.rightBumper().onTrue(driveToNearestReefSideCommandRight);
    joystickMoveTrigger.whileTrue(new InstantCommand(() -> driveToNearestReefSideCommandLeft.end(false))
        .alongWith(new InstantCommand(() -> driveToNearestReefSideCommandRight.end(false))));

    // driverController.leftTrigger().onTrue(new PrintCommand("running here!!!!"));
    /*
     * elevator.setDefaultCommand(
     * new ManualElevatorCommand(
     * elevator,
     * () -> operatorController.getLeftY()));
     */
    // operatorController.axisGreaterThan(0,
    // 0).onTrue(ElevatorCommands.testElevator(elevator));
    // driverController.a().whileTrue(DriveCommands.feedforwardCharacterization(drive));
    // m_driverController.a().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController.x().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // m_driverController.b().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController.y().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.get();
  }

  private boolean isJoystickMoved() {
    // Check if there's significant joystick movement
    return Math.abs(driverController.getLeftY()) > 0.5 ||
        Math.abs(driverController.getLeftX()) > 0.5 ||
        Math.abs(driverController.getRightX()) > 0.5 ||
        Math.abs(driverController.getRightY()) > 0.5;
    // return driverController.axisMagnitudeGreaterThan(0, 0.5).getAsBoolean();
  }
}
