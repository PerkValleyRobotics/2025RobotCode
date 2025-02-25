// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DeAlgifier.DeAlgifier;
import frc.robot.subsystems.DeAlgifier.DeAlgifierConstants;

/** Add your docs here. */
public class DeAlgifierCommands {

    public DeAlgifierCommands() {
    }

    public static Command toggleDealgifierCommand(DeAlgifier deAlgifier) {
        return Commands.runOnce(
                () -> {
                    deAlgifier.runWheel(true);
                    deAlgifier.gotoPos(DeAlgifierConstants.PIVOT_TARGET_LOCATION
                    );
                }).finallyDo(
                        () -> {
                            deAlgifier.stopWheel();
                            deAlgifier.gotoPos(0);
                        });

    }

}
