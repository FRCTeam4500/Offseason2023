package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

public class PlaceCommand extends SequentialCommandGroup{
    public PlaceCommand(Arm arm, Intake intake, GamePiece piece) {
        switch (piece) {
            case Cone:
                addCommands(
                    new SetIntakeSpeedCommand(intake, IntakeConstants.INTAKE_CUBE_SPEED),
                    new WaitCommand(1),
                    new SetIntakeSpeedCommand(intake, 0)
                );
                break;
            case Cube:
                addCommands(
                    new SetIntakeSpeedCommand(intake, IntakeConstants.INTAKE_CONE_SPEED),
                    new WaitCommand(1),
                    new SetIntakeSpeedCommand(intake, 0)
                );
                break;
            case Both:
                addCommands(
                    new SetIntakeSpeedCommand(intake, IntakeConstants.INTAKE_CONE_SPEED),
                    new WaitCommand(1),
                    new SetIntakeSpeedCommand(intake, IntakeConstants.INTAKE_CUBE_SPEED),
                    new WaitCommand(1),
                    new SetIntakeSpeedCommand(intake, 0)
                );
                break;
        }
    }

    public enum GamePiece {
        Cone, 
        Cube,
        Both
    }
}
