package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.EnumConstants.IntakeMode;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;

public class AutoPickupCommand extends SequentialCommandGroup {

	public AutoPickupCommand(IntakeMode mode) {
		addCommands(
			new SetArmAndIntakeCommand(ArmPosition.Bot),
			new SetIntakeSpeedCommand(mode)
		);
	}
}
