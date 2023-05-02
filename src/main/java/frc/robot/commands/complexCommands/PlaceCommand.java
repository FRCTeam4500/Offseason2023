package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand.Output;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

public class PlaceCommand extends SequentialCommandGroup{
    public PlaceCommand(Arm arm, Intake intake, GamePiece piece) {
        switch (piece) {
            case Cone:
                addCommands(
                    new SetIntakeSpeedCommand(intake, Output.PlaceCone),
                    new WaitCommand(1),
                    new SetIntakeSpeedCommand(intake, Output.Zero)
                );
                break;
            case Cube:
                addCommands(
                    new SetIntakeSpeedCommand(intake, Output.PlaceCube),
                    new WaitCommand(1),
                    new SetIntakeSpeedCommand(intake, Output.Zero)
                );
                break;
        }
    }

    public enum GamePiece {
        Cone, 
        Cube
    }
}
