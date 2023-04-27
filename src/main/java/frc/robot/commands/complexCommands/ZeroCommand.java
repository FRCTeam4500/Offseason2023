package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand.Position;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

public class ZeroCommand extends SequentialCommandGroup{
    public ZeroCommand(Arm arm, Intake intake) {
        addCommands(
            new SetIntakeSpeedCommand(intake, 0),
            new SetArmAndIntakeCommand(arm, intake, Position.Zero)
        );
    }
}
