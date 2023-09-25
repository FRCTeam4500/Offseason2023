package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EnumConstants.ArmPosition;

public class SetArmAndIntakeCommand extends SequentialCommandGroup {
	public SetArmAndIntakeCommand(ArmPosition position) {
		addCommands(
			new SetExtensionCommand(ArmPosition.Zero),
			new SetTiltCommand(position),
			new SetExtensionCommand(position)
		);
	}
}
