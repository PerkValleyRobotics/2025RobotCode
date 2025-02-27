// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorConstants;

/** Add your docs here. */
public class EndEffectorCommands {
    public EndEffectorCommands() {
    }

    public static Command runFrontMotors(EndEffector endEffector, boolean leftSlowed, boolean rightSlowed, double multiplier) {
        return Commands.run(() -> {
            endEffector.runFrontSpeed(EndEffectorConstants.FRONT_RIGHT_SPEED*multiplier);
        }, endEffector).finallyDo(
                () -> {
                    endEffector.stopFront();
                });
    }

    public static Command runBackCommand(EndEffector endEffector, double multiplier) {

        return Commands.run(() -> {
            endEffector.runBackSpeed(EndEffectorConstants.BACK_SPEED*multiplier);
        }, endEffector).finallyDo(() -> {
            endEffector.stopBack();
        });
    }
    
    public static Command runFrontAndBack(EndEffector endEffector, double multiplier) {
        return Commands.run(() -> {
            endEffector.runBackSpeed(0.25*multiplier);
            endEffector.runFrontSpeed(0.25*multiplier);
        }, endEffector).finallyDo(() -> {
            endEffector.stopBack();
            endEffector.stopFront();
        });
    }

    
}
