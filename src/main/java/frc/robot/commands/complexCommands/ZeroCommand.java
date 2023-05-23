package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EnumConstants.IntakeSpeed;
import frc.robot.Constants.EnumConstants.PlacerState;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

public class ZeroCommand extends SequentialCommandGroup {

	public ZeroCommand(Arm arm, Intake intake) {
		addCommands(
			new SetIntakeSpeedCommand(intake, IntakeSpeed.Off),
			new SetArmAndIntakeCommand(arm, intake, PlacerState.Zero)
		);
	}
}
