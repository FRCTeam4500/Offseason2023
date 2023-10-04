package frc.robot.autonomous.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.EnumConstants.IntakeMode;
import frc.robot.commands.autoCommands.AutoTimedDriveCommand;
import frc.robot.commands.baseCommands.ResetGyroCommand;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.complexCommands.AutoPickupCommand;
import frc.robot.commands.complexCommands.ZeroCommand;

public class TwoPieceAuto extends SequentialCommandGroup {

	public TwoPieceAuto() {
		addCommands(
			new ResetGyroCommand(180),
			new ZeroCommand(),
			new WaitCommand(0.5),
			new SetArmAndIntakeCommand(ArmPosition.Top),
			new WaitCommand(0.75),
			new SetIntakeSpeedCommand(IntakeMode.Place),
			new WaitCommand(1),
			new ZeroCommand(),
			new WaitCommand(0.5),
			new AutoTimedDriveCommand(new ChassisSpeeds(-2, 0, 0), 2),
			new AutoTimedDriveCommand(new ChassisSpeeds(0, 0, 2 * Math.PI), 2),
			new AutoPickupCommand(IntakeMode.PickupCube),
			new WaitCommand(1),
			new AutoTimedDriveCommand(new ChassisSpeeds(1, 0, 0), 1),
			new ZeroCommand(),
			new WaitCommand(1),
			new AutoTimedDriveCommand(new ChassisSpeeds(0, 0, 2 * Math.PI), 2),
			new AutoTimedDriveCommand(new ChassisSpeeds(2, 0, 0), 2)
		);
	}
}
