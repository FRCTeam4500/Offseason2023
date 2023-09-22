package frc.robot.autonomous.paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutonomousDriveCommand;
import frc.robot.autonomous.AutonomousDriveCommand.AutonomousDriveStage;
import frc.robot.autonomous.AutonomousDriveCommand.StageType;

public class TestPath1 {

	public static AutonomousDriveCommand getPathCommand() {
		return new AutonomousDriveCommand(
			new AutonomousDriveStage(
				StageType.FOLLOW_POSE_2D_RELATIVE,
				new Pose2d(new Translation2d(2, 0), new Rotation2d(0)),
				2,
				0,
				2,
				2,
				2,
				2
			)
		);
	}
}
