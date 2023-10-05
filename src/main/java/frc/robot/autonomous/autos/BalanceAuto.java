package frc.robot.autonomous.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoCommands.AutoTimedDriveCommand;
import frc.robot.commands.baseCommands.ResetGyroCommand;
import frc.robot.commands.complexCommands.AutoBalanceCommand;
import frc.robot.commands.complexCommands.TeleopZeroCommand;
import frc.robot.commands.complexCommands.ZeroCommand;

public class BalanceAuto extends SequentialCommandGroup {

	public BalanceAuto() {
		addCommands(
			new ResetGyroCommand(180),
			// new ZeroCommand(), FIXME: Why No Zero?
			new AutoTimedDriveCommand(new ChassisSpeeds(-2, 0, 0), 2.25),
			new AutoTimedDriveCommand(new ChassisSpeeds(2, 0, 0), 1.25),
			new AutoBalanceCommand()
		);
	}
}
