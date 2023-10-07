package frc.robot.autonomous.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.EnumConstants.IntakeMode;
import frc.robot.commands.autoCommands.AutoTimedDriveCommand;
import frc.robot.commands.baseCommands.ResetGyroCommand;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.complexCommands.AutoBalanceCommand;
import frc.robot.commands.complexCommands.TeleopZeroCommand;

public class BalanceAuto extends SequentialCommandGroup {
	public BalanceAuto() {
		addCommands(
			new ResetGyroCommand(180),
			new SetArmAndIntakeCommand(ArmPosition.Start), 
			new WaitCommand(0.5),
			new SetArmAndIntakeCommand(ArmPosition.Top),
			new WaitCommand(1.25),
			new SetIntakeSpeedCommand(IntakeMode.Place),
			new WaitCommand(1),
			new TeleopZeroCommand(),
			new WaitCommand(0.5),
			new AutoTimedDriveCommand(-2, 0, 0, 2.25),
			new AutoTimedDriveCommand(2, 0, 0, 1.25),
			new AutoBalanceCommand()
		);
	}
}
