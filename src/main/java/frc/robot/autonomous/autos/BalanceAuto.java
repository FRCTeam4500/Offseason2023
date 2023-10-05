package frc.robot.autonomous.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.complexCommands.SwerveDriveCommand;
import frc.robot.commands.complexCommands.TimedDriveCommand;
import frc.robot.Constants.EnumConstants.ControlMode;
import frc.robot.commands.baseCommands.ResetGyroCommand;
import frc.robot.commands.complexCommands.ZeroCommand;

public class BalanceAuto extends SequentialCommandGroup {

	public BalanceAuto() {
		addCommands(
			new ResetGyroCommand(180),
			new ZeroCommand(), 
			new TimedDriveCommand(new ChassisSpeeds(-2, 0, 0), 2.25),
			new TimedDriveCommand(new ChassisSpeeds(2, 0, 0), 1.25),
			new InstantCommand(() -> SwerveDriveCommand.getInstance().setControlMode(ControlMode.Balance))
		);
	}
}
