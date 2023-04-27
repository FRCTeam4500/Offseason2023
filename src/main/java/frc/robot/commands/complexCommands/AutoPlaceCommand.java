package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand.Position;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

public class AutoPlaceCommand extends SequentialCommandGroup{
    public AutoPlaceCommand(Arm arm, Intake intake, Location position) {
        switch (position) {
            case HighCone:
                addCommands(
                    new SetArmAndIntakeCommand(arm, intake, Position.High),
                    new WaitCommand(1),
                    new SetIntakeSpeedCommand(intake, IntakeConstants.INTAKE_CUBE_SPEED),
                    new WaitCommand(1),
                    new ZeroCommand(arm, intake)
                );
                break;
            case MidCone:
                addCommands(
                    new SetArmAndIntakeCommand(arm, intake, Position.Middle),
                    new WaitCommand(1),
                    new SetIntakeSpeedCommand(intake, IntakeConstants.INTAKE_CUBE_SPEED),
                    new WaitCommand(1),
                    new ZeroCommand(arm, intake)
                );
                break;
            case HighCube:
                addCommands(
                    new SetArmAndIntakeCommand(arm, intake, Position.High),
                    new WaitCommand(1),
                    new SetIntakeSpeedCommand(intake, IntakeConstants.INTAKE_CONE_SPEED),
                    new WaitCommand(1),
                    new ZeroCommand(arm, intake)
                );
                break;
            case MidCube:
                addCommands(
                    new SetArmAndIntakeCommand(arm, intake, Position.Middle),
                    new WaitCommand(1),
                    new SetIntakeSpeedCommand(intake, IntakeConstants.INTAKE_CONE_SPEED),
                    new WaitCommand(1),
                    new ZeroCommand(arm, intake)
                );
                break;
        }
    }
    public enum Location {
        HighCone,
        MidCone,
        HighCube,
        MidCube
    }
}
