// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.Vision.VisionConstants.*;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.fasterxml.jackson.databind.node.IntNode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToHPStationCommand;
import frc.robot.commands.DriveToNearestReefSideCommand;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DeAlgifierCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.ElevatorGotoHeightCommand;
import frc.robot.commands.EndEffectorCommands;
import frc.robot.commands.ManualElevatorCommand;
// import frc.robot.subsystems.CoralSensor.CoralSensor;
// import frc.robot.subsystems.CoralSensor.CoralSensorIO;
// import frc.robot.subsystems.CoralSensor.CoralSensorIOReal;
// import frc.robot.subsystems.CoralSensor.CoralSensorIOSim;
import frc.robot.subsystems.DeAlgifier.DeAlgifier;
import frc.robot.subsystems.DeAlgifier.DeAlgifierIO;
import frc.robot.subsystems.DeAlgifier.DeAlgifierIOSparkMax;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.ModuleIO;
import frc.robot.subsystems.Drive.ModuleIOSim;
import frc.robot.subsystems.Drive.ModuleIOSparkMax;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorIO;
import frc.robot.subsystems.EndEffector.EndEffectorIOSparkMax;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIONavx;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOSparkMax;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOLimelight;
import frc.robot.subsystems.Vision.VisionIOPhotonVisionSim;

import static frc.robot.IsDetectionAllowed.*;

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
        public final Elevator elevator;
        // private final CoralSensor coralSensor;
        private final EndEffector endEffector;
        private final DeAlgifier deAlgifier;
        private final Intake intake;

        private final CommandXboxController driverController = new CommandXboxController(0);
        private final Trigger joystickMoveTrigger = new Trigger(() -> isJoystickMoved());
        private final CommandXboxController operatorController = new CommandXboxController(1);

        private final LoggedDashboardChooser<Command> autoChooser;

        private BooleanSupplier isAutoAligning = () -> false;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                boolean COMPETITION_MODE = true;

                switch (Constants.currentMode) {
                        case REAL:
                                // coralSensor = new CoralSensor(new CoralSensorIOReal());
                                drive = new Drive(
                                                new GyroIONavx(),
                                                new ModuleIOSparkMax(0),
                                                new ModuleIOSparkMax(1),
                                                new ModuleIOSparkMax(2),
                                                new ModuleIOSparkMax(3));
                                vision = new Vision(
                                                drive::addVisionMeasurement,
                                                new VisionIOLimelight("limelight-front", drive::getRotation));
                                elevator = new Elevator(new ElevatorIOSparkMax());

                                endEffector = new EndEffector(new EndEffectorIOSparkMax());
                                deAlgifier = new DeAlgifier(new DeAlgifierIOSparkMax());

                                intake = new Intake(new IntakeIOSparkMax());

                                break;

                        case SIM:
                                // coralSensor = new CoralSensor(new CoralSensorIOSim());
                                drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim());
                                vision = new Vision(
                                                drive::addVisionMeasurement,
                                                new VisionIOPhotonVisionSim("limelight", robotToCamera,
                                                                drive::getPose));
                                elevator = new Elevator(
                                                new ElevatorIOSim());

                                endEffector = new EndEffector(new EndEffectorIO() {
                                });
                                deAlgifier = new DeAlgifier(new DeAlgifierIO() {
                                });

                                intake = new Intake(new IntakeIO() {
                                });
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
                                // coralSensor = new CoralSensor(new CoralSensorIO() {
                                // });
                                endEffector = new EndEffector(new EndEffectorIO() {
                                });
                                deAlgifier = new DeAlgifier(new DeAlgifierIO() {
                                });

                                intake = new Intake(new IntakeIO() {
                                });
                                break;
                }

                // elevator auton
                NamedCommands.registerCommand("GoToHome", new InstantCommand(elevator::home));
                NamedCommands.registerCommand("GoToL1", new InstantCommand(elevator::gotoL1));
                NamedCommands.registerCommand("GoToL2", new InstantCommand(elevator::gotoL2));
                NamedCommands.registerCommand("GoToL3", new InstantCommand(elevator::gotoL3));
                NamedCommands.registerCommand("GoToL4", new InstantCommand(elevator::gotoL4));
                NamedCommands.registerCommand("RunFrontAndBack",
                                EndEffectorCommands.runFrontAndBack(endEffector, 1).withTimeout(.5));
                NamedCommands.registerCommand("RunBackMotor",
                                EndEffectorCommands.runBackCommand(endEffector, .8).withTimeout(1.2)
                                                .finallyDo(() -> EndEffectorCommands.runFrontAndBack(endEffector, -1)
                                                                .withTimeout(0.05)));
                NamedCommands.registerCommand("StartAndStopDetection",
                                new InstantCommand(() -> isDetectionAllowed = true)
                                                .withTimeout(.2).withTimeout(.2)
                                                .finallyDo(() -> new InstantCommand(() -> isDetectionAllowed = false)));
                NamedCommands.registerCommand("RunDeAlgifier",
                                new SequentialCommandGroup(new InstantCommand(deAlgifier::out),
                                                new InstantCommand(deAlgifier::runWheel)).withTimeout(1.0));
                autoChooser = new LoggedDashboardChooser<>("Auto Choices",
                                AutoBuilder.buildAutoChooserWithOptionsModifier(
                                                (stream) -> COMPETITION_MODE ? stream.filter(
                                                                auto -> auto.getName().startsWith("(comp)")) : stream));

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

                // reef auto drive binds
                // reef auto drive binds
                Command driveToNearestReefSideCommandLeft = new DriveToNearestReefSideCommand(drive,
                                true, false);
                Command driveToNearestReefSideCommandRight = new DriveToNearestReefSideCommand(drive,
                                false, false);
                driverController.leftBumper().onTrue(driveToNearestReefSideCommandLeft);
                driverController.rightBumper().onTrue(driveToNearestReefSideCommandRight);

                Command driveToNearestReefSideCommandCenter = new DriveToNearestReefSideCommand(drive,
                                false, true);
                driverController.pov(0).onTrue(driveToNearestReefSideCommandCenter);
                // hp station auto drive binds
                // hp station auto drive binds
                // Command driveToHPStationCommand = new DriveToHPStationCommand(drive);
                // driverController.a().onTrue(driveToHPStationCommand);
                // stop auto drive when move joysticks
                joystickMoveTrigger.or(driverController.pov(180))
                                .whileTrue(new InstantCommand(() -> driveToNearestReefSideCommandLeft.end(false))
                                                .alongWith(new InstantCommand(
                                                                () -> driveToNearestReefSideCommandRight.end(false)))
                                                .alongWith(new InstantCommand(
                                                                () -> driveToNearestReefSideCommandCenter.end(false))));
                // .alongWith(new InstantCommand(() -> driveToHPStationCommand.end(false))));

                drive.setDefaultCommand(
                                DriveCommands.FPSDrive(
                                                drive,
                                                () -> -driverController.getLeftY(),
                                                () -> -driverController.getLeftX(),
                                                () -> -driverController.getRightX()).finallyDo(() -> {
                                                        new InstantCommand(() -> drive.stop());
                                                }));
                driverController
                                .y()
                                .whileTrue(
                                                DriveCommands.FPSDrive(
                                                                drive,
                                                                () -> -driverController.getLeftY() / 2,
                                                                () -> -driverController.getLeftX() / 2,
                                                                () -> -driverController.getRightX() / 2));

                // strafe with triggers
                driverController.leftTrigger(0.25).whileTrue(DriveCommands.RelativeDrive(
                                drive,
                                () -> 0,
                                () -> driverController.getLeftTriggerAxis() * 0.5,
                                () -> 0));
                driverController.rightTrigger(0.25).whileTrue(DriveCommands.RelativeDrive(
                                drive,
                                () -> 0,
                                () -> -driverController.getRightTriggerAxis() * 0.5,
                                () -> 0));
                // driverController.leftTrigger(0.25).whileTrue(new InstantCommand(() ->
                // drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(new
                // ChassisSpeeds(driverController.getLeftTriggerAxis(), 0,
                // Units.degreesToRadians(90))))));
                // m_driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
                // driverController
                // .x()
                // .onTrue(
                // Commands.runOnce(
                // () -> drive.setPose(
                // new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                // drive)
                // .ignoringDisable(true));
                driverController
                                .b()
                                .onTrue(
                                                Commands.runOnce(
                                                                () -> drive.setPose(
                                                                                new Pose2d(new Translation2d(),
                                                                                                new Rotation2d())),
                                                                drive)
                                                                .ignoringDisable(true));

                driverController
                                .x()
                                .whileTrue(
                                                DriveCommands.feedforwardCharacterization(drive));

                // Elevator setpoints
                operatorController
                                .a()
                                .onTrue(
                                                new InstantCommand(elevator::home));
                operatorController
                                .b()
                                .onTrue(
                                                new InstantCommand(elevator::gotoL1));
                operatorController
                                .x()
                                .onTrue(
                                                new InstantCommand(elevator::gotoL2));
                operatorController
                                .y()
                                .onTrue(
                                                new InstantCommand(elevator::gotoL3));

                // Manual elevator control
                // operatorController
                // .pov(0)
                // .whileTrue(
                // new RepeatCommand(new InstantCommand(elevator::incrementSetpoint))
                // );

                // operatorController
                // .pov(180)
                // .whileTrue(
                // new RepeatCommand(new InstantCommand(elevator::decrementSetpoint))
                // );

                // End effector binds
                operatorController.leftBumper().whileTrue(EndEffectorCommands.runFrontAndBack(endEffector, 1));
                operatorController.rightBumper().whileTrue(EndEffectorCommands.runBackCommand(endEffector, 1));
                // operatorController.a().whileTrue(EndEffectorCommands.runFrontMotors(endEffector,
                // false, false, 1));

                // operatorController.leftBumper().and(operatorController.back())
                //                 .whileTrue(EndEffectorCommands.runFrontAndBack(endEffector, -1));
                // operatorController.rightBumper().and(operatorController.back())
                                // .whileTrue(EndEffectorCommands.runBackCommand(endEffector, -.5));
                operatorController.pov(180).whileTrue(EndEffectorCommands.runFrontAndBack(endEffector, .9));



                operatorController.button(8).whileTrue(new InstantCommand(() -> intake.goBack())); // operatorController.a().and(operatorController.back()).whileTrue(EndEffectorCommands.runFrontMotors(endEffector,
                operatorController.button(8).and(operatorController.back())
                                .whileTrue(new InstantCommand(() -> intake.goHome()));
                // false, false, -1));
                // operatorController.a().and(() -> !(coralSensor.getDistance() <
                // 2)).whileTrue(EndEffectorCommands.runFrontMotors(endEffector, false, false));

                // // dealgifier binds
                // operatorController
                // .pov(90)
                // .whileTrue(
                // DeAlgifierCommands.toggleDealgifierCommand(deAlgifier));

                // operatorController
                // .pov(270)
                // .whileTrue(
                // new InstantCommand(deAlgifier::in));

                operatorController
                                .pov(270)
                                .onTrue(
                                                new SequentialCommandGroup(new InstantCommand(deAlgifier::out),
                                                                new InstantCommand(deAlgifier::runWheel)));
                operatorController
                                .pov(90)
                                .onTrue(
                                                new SequentialCommandGroup(new InstantCommand(deAlgifier::in),
                                                                new InstantCommand(deAlgifier::stopWheel)));

                // operatorController
                // .pov(270)
                // .whileTrue(
                // new RepeatCommand(new InstantCommand(deAlgifier::decrementSetpoint))
                // );

                // operatorController.pov(270).whileTrue(new
                // InstantCommand(deAlgifier::setblahblah));
                // operatorController.b().toggleOnTrue(DeAlgifierCommands.toggleDealgifierCommand(deAlgifier));

                // operatorController
                // .b()
                // .onTrue(
                // new InstantCommand(elevator::home)
                // );
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
                return Math.abs(driverController.getLeftY()) > 0.2 ||
                                Math.abs(driverController.getLeftX()) > 0.2 ||
                                Math.abs(driverController.getRightX()) > 0.2 ||
                                Math.abs(driverController.getRightY()) > 0.2;
                // return driverController.axisMagnitudeGreaterThan(0, 0.5).getAsBoolean();
        }
}