package frc.robot.autonomous.routines;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EnumConstants.IntakeMode;
import frc.robot.commands.autoCommands.AutoTimedDriveCommand;
import frc.robot.commands.complexCommands.AutoPickupCommand;
import frc.robot.commands.complexCommands.ZeroCommand;

public class TestGroundPickup extends SequentialCommandGroup {

	public TestGroundPickup() {
		addCommands(
			new AutoPickupCommand(IntakeMode.PickupCone)
				.alongWith(
					new AutoTimedDriveCommand(new ChassisSpeeds(1, 0, 0), 3)
				),
			new ZeroCommand()
		);
	}
}
