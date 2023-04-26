package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand.Position;
import frc.robot.commands.complexCommands.PlaceCommand.GamePiece;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

public class AutoPickupCommand extends SequentialCommandGroup{
    public AutoPickupCommand(Arm arm, Intake intake, Piece piece) {
        switch (piece) {
            case Cube:
                addCommands(
                    new SetArmAndIntakeCommand(arm, intake, Position.Low),
                    new SetIntakeSpeedCommand(intake, IntakeConstants.INTAKE_CUBE_SPEED)
                );
                break;
            case Cone:
                addCommands(
                    new SetArmAndIntakeCommand(arm, intake, Position.Low),
                    new SetIntakeSpeedCommand(intake, IntakeConstants.INTAKE_CONE_SPEED)
                );
                break;
        }
    }
    public enum Piece {
        Cube,
        Cone
    }
}
