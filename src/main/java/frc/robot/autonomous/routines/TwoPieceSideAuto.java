package frc.robot.autonomous.routines;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.EnumConstants.IntakeMode;
import frc.robot.Constants.EnumConstants.VisionTarget;
import frc.robot.commands.autoCommands.AutoAlignCommand;
import frc.robot.commands.autoCommands.AutoAlignHorizontalCommand;
import frc.robot.commands.autoCommands.AutoAlignParallelCommand;
import frc.robot.commands.autoCommands.AutoDriveToCommand;
import frc.robot.commands.autoCommands.AutoTimedDriveCommand;
import frc.robot.commands.complexCommands.AutoPickupCommand;
import frc.robot.commands.complexCommands.AutoPlaceCommand;
import frc.robot.commands.complexCommands.ZeroCommand;

public class TwoPieceSideAuto extends SequentialCommandGroup {

	public TwoPieceSideAuto() {
		addCommands(
			new AutoPlaceCommand(ArmPosition.Top),
			new AutoTimedDriveCommand(new ChassisSpeeds(-1, 0, 0), 3), // TODO: Change speed and time --> Initial drive back
			new AutoTimedDriveCommand(
				new ChassisSpeeds(-1, 0, -Math.PI / 2),
				2
			), // TODO: Change speed and time --> Turn to face piece -> Could cause issue where robot does a nice left turn
			new AutoPickupCommand(IntakeMode.PickupCube),
			new AutoTimedDriveCommand(new ChassisSpeeds(1, 0, 0), 1), // Drive Forward Until Picked Up
			new ZeroCommand(),
			new AutoTimedDriveCommand(
				new ChassisSpeeds(-1, 0, -Math.PI / 2),
				3
			), // TODO: Change speed and time --> Turn to face the grid
			new AutoTimedDriveCommand(new ChassisSpeeds(1, 0, 0), 1.5), // Drive until april tags visible
			new AutoAlignParallelCommand(), // Rotate to face parallel to grid
			new AutoAlignHorizontalCommand(VisionTarget.AprilTag), // Align to horizontal Crab walk
			new AutoDriveToCommand(VisionTarget.AprilTag), // Drive to april tag
			new AutoAlignHorizontalCommand(VisionTarget.AprilTag), // Align to horizontal Crab walk
			new AutoTimedDriveCommand(new ChassisSpeeds(0, 1, 0), 1), // Move to a cube node
			new AutoPlaceCommand(ArmPosition.Top)
		);
	}
}
