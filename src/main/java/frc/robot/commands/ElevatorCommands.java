package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;

public class ElevatorCommands {

    public ElevatorCommands() {}

    public static Command testElevator(Elevator elevator) {
        return Commands.sequence(
            new ElevatorGotoHeightCommand(elevator, ElevatorConstants.L1_HEIGHT),
            new ElevatorGotoHeightCommand(elevator, 0),
            new ElevatorGotoHeightCommand(elevator, ElevatorConstants.L2_HEIGHT),
            new ElevatorGotoHeightCommand(elevator, 0),
            new ElevatorGotoHeightCommand(elevator, ElevatorConstants.L3_HEIGHT),
            new ElevatorGotoHeightCommand(elevator, 0),
            new ElevatorGotoHeightCommand(elevator, ElevatorConstants.L4_HEIGHT),
            new ElevatorGotoHeightCommand(elevator, 0)
        );
    }
}
