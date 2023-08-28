package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.EnumConstants.IntakeMode;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;

public class PlaceCommand extends SequentialCommandGroup {

	public PlaceCommand() {
		addCommands(
			new SetIntakeSpeedCommand(IntakeMode.Place),
			new WaitCommand(1),
			new SetIntakeSpeedCommand(IntakeMode.Off)
		);
	}
}
