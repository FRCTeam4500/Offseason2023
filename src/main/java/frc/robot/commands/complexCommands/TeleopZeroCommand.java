package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.EnumConstants.IntakeMode;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;

public class TeleopZeroCommand extends SequentialCommandGroup {

	public TeleopZeroCommand() {
		addCommands(
			new SetIntakeSpeedCommand(IntakeMode.Off),
			new SetArmAndIntakeCommand(ArmPosition.TELEOP_MOVING)
		);
	}
}
