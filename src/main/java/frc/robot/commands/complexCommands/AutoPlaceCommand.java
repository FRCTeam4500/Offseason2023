package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.EnumConstants.IntakeMode;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;

public class AutoPlaceCommand extends SequentialCommandGroup {

	public AutoPlaceCommand(ArmPosition position) {
		addCommands(
			new SetArmAndIntakeCommand(position),
			new WaitCommand(1),
			new SetIntakeSpeedCommand(IntakeMode.Place),
			new WaitCommand(1),
			new ZeroCommand()
		);
	}
}
