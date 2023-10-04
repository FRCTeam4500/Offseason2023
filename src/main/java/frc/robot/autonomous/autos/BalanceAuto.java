package frc.robot.autonomous.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoCommands.AutoTimedDriveCommand;
import frc.robot.commands.baseCommands.ResetGyroCommand;
import frc.robot.commands.complexCommands.AutoBalanceCommand;
import frc.robot.commands.complexCommands.ZeroCommand;

public class BalanceAuto extends SequentialCommandGroup {

	public BalanceAuto() {
		addCommands(
			new ResetGyroCommand(180),
			// new ZeroCommand(),
			new AutoTimedDriveCommand(new ChassisSpeeds(-1.5, 0, 0), 5),
			new AutoTimedDriveCommand(new ChassisSpeeds(1, 0, 0), 4),
			new AutoBalanceCommand()
		);
	}
}
