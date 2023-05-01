package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand.Position;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand.Output;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

public class ZeroCommand extends SequentialCommandGroup{
    public ZeroCommand(Arm arm, Intake intake) {
        addCommands(
            new SetIntakeSpeedCommand(intake, Output.Zero),
            new SetArmAndIntakeCommand(arm, intake, Position.Zero)
        );
    }
}
