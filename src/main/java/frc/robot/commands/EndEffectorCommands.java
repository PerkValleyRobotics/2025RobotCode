// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.EndEffector.EndEffector;

/** Add your docs here. */
public class EndEffectorCommands {
    public EndEffectorCommands() {
    }

    public static Command runFrontMotors(EndEffector endEffector, boolean leftSlowed, boolean rightSlowed) {
        return Commands.run(() -> {
            endEffector.runLeft(leftSlowed);
            endEffector.runRight(rightSlowed);
        }, endEffector).finallyDo(
                () -> {
                    endEffector.stopFront();
                });
    }

    public static Command runBackCommand(EndEffector endEffector) {
        return Commands.run(() -> {
            endEffector.runBack();
        }, endEffector).finallyDo(() -> {
            endEffector.stopBack();
        });
    }
    
    public static Command runFrontAndBack(EndEffector endEffector) {
        return Commands.sequence(runBackCommand(endEffector), runFrontMotors(endEffector, false, false));
    }
}
