package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.complexCommands.AutoAlignHorizontalCommand;
import frc.robot.commands.complexCommands.AutoAlignRotationalCommand;
import frc.robot.commands.complexCommands.AutoDriveToCommand;
import frc.robot.subsystem.swerve.SwerveDrive;
import frc.robot.subsystem.vision.Vision;

public class AutonomousDriveCommand extends CommandBase {

	private int currentStage = 0;
	private AutonomousDriveStage[] stages;
	private SwerveDrive swerve;
	private Vision vision;
	private PIDController translationPID = new PIDController(1, 0, 0);
	private PIDController rotationPID = new PIDController(1, 0, 0);

	public AutonomousDriveCommand(AutonomousDriveStage... stages) {
		this.stages = stages;
		this.swerve = SwerveDrive.getInstance();
		this.vision = Vision.getInstance();
	}

	public void driveToPose2d() {}

	public void driveToTarget() {}

	@Override
	public void execute() {
		Pose2d targetPose;
		Pose2d currentPose;
		double xSpeed;
		double ySpeed;
		double rotationalSpeed;

		AutonomousDriveStage stage = stages[currentStage];
		switch (stage.type) {
			case VISION_ALIGN_TARGET_ROTATION:
				if (vision.hasValidTargets(stage.limelightId)) {
					CommandScheduler
						.getInstance()
						.schedule(
							new AutoAlignRotationalCommand(
								stage.limelightId,
								stage.timeThreshold,
								stage.rotationalThreshold
							)
						);
				} else {
					break;
				}
			case VISION_ALIGN_TARGET_TRANSLATION:
				if (vision.hasValidTargets(stage.limelightId)) {
					CommandScheduler
						.getInstance()
						.schedule(
							new SequentialCommandGroup(
								new AutoDriveToCommand(stage.limelightId),
								new AutoAlignHorizontalCommand(
									stage.limelightId,
									stage.timeThreshold,
									stage.xThreshold
								)
							)
						);
				} else {
					break;
				}
			case FOLLOW_POSE_2D_RELATIVE:
				targetPose = stage.pose;
				currentPose = swerve.getRobotPose();

				xSpeed =
					translationPID.calculate(
						currentPose.getTranslation().getX(),
						targetPose.getTranslation().getX()
					);
				ySpeed =
					translationPID.calculate(
						currentPose.getTranslation().getY(),
						targetPose.getTranslation().getY()
					);
				rotationalSpeed =
					rotationPID.calculate(
						currentPose.getRotation().getRadians(),
						targetPose.getRotation().getRadians()
					);

				swerve.driveRobotCentric(
					xSpeed / 2,
					ySpeed / 2,
					rotationalSpeed / 10
				);

				break;
			case FOLLOW_POSE_2D_FIELD:
				targetPose = stage.pose;
				currentPose = swerve.getRobotPose();

				xSpeed =
					translationPID.calculate(
						currentPose.getTranslation().getX(),
						targetPose.getTranslation().getX()
					);
				ySpeed =
					translationPID.calculate(
						currentPose.getTranslation().getY(),
						targetPose.getTranslation().getY()
					);
				rotationalSpeed =
					rotationPID.calculate(
						currentPose.getRotation().getRadians(),
						targetPose.getRotation().getRadians()
					);

				swerve.driveFieldCentric(
					xSpeed / 2,
					ySpeed / 2,
					rotationalSpeed / 10
				);

				break;
			default:
				break;
		}
	}

	public enum StageType {
		VISION_ALIGN_TARGET_ROTATION,
		VISION_ALIGN_TARGET_TRANSLATION,
		FOLLOW_POSE_2D_RELATIVE,
		FOLLOW_POSE_2D_FIELD,
	}

	public class AutonomousDriveStage {

		private StageType type;
		private Pose2d pose;

		private double xThreshold;
		private double yThreshold;
		private double rotationalThreshold;
		private int timeThreshold;
		private double speed;

		private int limelightId;

		public AutonomousDriveStage(
			StageType type,
			double speed,
			int limelightId,
			double xThreshold,
			double yThreshold,
			double rotationalThreshold,
			int timeThreshold
		) {
			this.type = type;
			this.speed = speed;
			this.xThreshold = xThreshold;
			this.yThreshold = yThreshold;
			this.rotationalThreshold = rotationalThreshold;
			this.timeThreshold = timeThreshold;
			this.limelightId = limelightId;
		}

		public AutonomousDriveStage(
			StageType type,
			Pose2d pose,
			double speed,
			int limelightId,
			double xThreshold,
			double yThreshold,
			double rotationalThreshold,
			int timeThreshold
		) {
			this.pose = pose;
			this.type = type;
			this.speed = speed;
			this.xThreshold = xThreshold;
			this.yThreshold = yThreshold;
			this.rotationalThreshold = rotationalThreshold;
			this.timeThreshold = timeThreshold;
			this.limelightId = limelightId;
		}

		public AutonomousDriveStage(
			Pose2d pose,
			double speed,
			int limelightId,
			double xThreshold,
			double yThreshold,
			double rotationalThreshold,
			int timeThreshold
		) {
			this.pose = pose;
			this.type = StageType.FOLLOW_POSE_2D_RELATIVE;
			this.speed = speed;
			this.xThreshold = xThreshold;
			this.yThreshold = yThreshold;
			this.rotationalThreshold = rotationalThreshold;
			this.timeThreshold = timeThreshold;
			this.limelightId = limelightId;
		}

		public Pose2d getPose() {
			return pose;
		}

		public StageType getType() {
			return type;
		}

		public double getXThreshold() {
			return xThreshold;
		}

		public double getYThreshold() {
			return yThreshold;
		}

		public double getTimeThreshold() {
			return timeThreshold;
		}

		public double getSpeed() {
			return speed;
		}
	}
}
