package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand.Position;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand.Output;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

public class AutoPickupCommand extends SequentialCommandGroup{
    public AutoPickupCommand(Arm arm, Intake intake, Piece piece) {
        switch (piece) {
            case Cube:
                addCommands(
                    new SetArmAndIntakeCommand(arm, intake, Position.Low),
                    new SetIntakeSpeedCommand(intake, Output.PickupCube)
                );
                break;
            case Cone:
                addCommands(
                    new SetArmAndIntakeCommand(arm, intake, Position.Low),
                    new SetIntakeSpeedCommand(intake, Output.PickupCone)
                );
                break;
        }
    }
    public enum Piece {
        Cube,
        Cone
    }
}
