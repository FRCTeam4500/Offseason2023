package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EnumConstants.VisionTarget;
import frc.robot.subsystems.vision.Vision;

public class AutoAlignCommand extends SequentialCommandGroup {

	public AutoAlignCommand(VisionTarget target) {
		double targetAngle = 180;
		switch (target) {
			case GamePiece:
				addCommands(
					new AutoAlignRotationalCommand(target),
					new AutoDriveToCommand(target)
				);
				break;
			case AprilTag:
				Vision
					.getInstance()
					.setPipeline(target.limelightId, target.pipeline);
				int ID = Vision.getInstance().getTagId(target.limelightId);
				if (ID == 4 || ID == 5) {
					targetAngle = 0;
				}
				addCommands();
			case ReflectiveTape:
				addCommands(
					new AutoTurnCommand(targetAngle),
					new AutoAlignHorizontalCommand(target)
				);
				break;
		}
	}
}
