package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.swerve.SwerveDrive;
import frc.robot.subsystem.vision.Vision;
import frc.robot.utility.ExtendedMath;

public class AutonomousDriveCommand extends CommandBase {

	private int currentStage = 0;
	private AutonomousDriveStage[] stages;
	private SwerveDrive swerve;
	private Vision vision;
	private PIDController translationPID = new PIDController(1, 0, 0);
	private PIDController rotationPID = new PIDController(1, 0, 0);
	AutonomousDriveStage stage;

	Pose2d targetPose;
	Pose2d currentPose;
	double xSpeed;
	double ySpeed;
	double rotationalSpeed;
	int timesCorrect = 0;
	boolean finished = false;

	public AutonomousDriveCommand(AutonomousDriveStage... stages) {
		this.stages = stages;
		this.stage = stages[0];
		this.swerve = SwerveDrive.getInstance();
		this.vision = Vision.getInstance();
	}

	public void driveToPose2d() {}

	public void driveToTarget() {}

	@Override
	public void execute() {
		switch (stage.getType()) {
			case VISION_ALIGN_TARGET_ROTATION:
				if (vision.hasValidTargets(stage.getLimelightId())) {
					double horizontalAngleOffset = Units.radiansToDegrees(
						vision.getHorizontalAngleOffset(stage.getLimelightId())
					);
					swerve.driveRobotCentric(
						0,
						0,
						ExtendedMath.clamp(
							-stage.speed,
							stage.speed,
							rotationPID.calculate(horizontalAngleOffset) / 2
						)
					);
					if (
						Math.abs(horizontalAngleOffset) <
						stage.getRotationalThreshold()
					) {
						timesCorrect++;
					} else {
						timesCorrect = 0;
					}
				} else {
					break;
				}
			case VISION_ALIGN_TARGET_TRANSLATION:
				if (vision.hasValidTargets(stage.getLimelightId())) {
					double horizontalAngleOffset = Units.radiansToDegrees(
						vision.getHorizontalAngleOffset(stage.getLimelightId())
					);
					swerve.driveRobotCentric(
						0,
						ExtendedMath.clamp(
							-stage.speed,
							stage.speed,
							translationPID.calculate(horizontalAngleOffset) / 2
						),
						0
					);
					if (
						Math.abs(horizontalAngleOffset) <
						stage.getRotationalThreshold()
					) {
						timesCorrect++;
					} else {
						timesCorrect = 0;
					}
				} else {
					break;
				}
			case FOLLOW_POSE_2D_RELATIVE:
				targetPose = stage.getPose();
				currentPose = swerve.getRobotPose();

				xSpeed =
					ExtendedMath.clamp(
						-stage.speed,
						stage.speed,
						translationPID.calculate(
							currentPose.getTranslation().getX(),
							targetPose.getTranslation().getX()
						)
					);
				ySpeed =
					ExtendedMath.clamp(
						-stage.speed,
						stage.speed,
						translationPID.calculate(
							currentPose.getTranslation().getY(),
							targetPose.getTranslation().getY()
						) /
						2
					);
				rotationalSpeed =
					ExtendedMath.clamp(
						-stage.speed,
						stage.speed,
						rotationPID.calculate(
							currentPose.getRotation().getRadians(),
							targetPose.getRotation().getRadians()
						) /
						2
					);

				swerve.driveRobotCentric(xSpeed, ySpeed, rotationalSpeed / 3);

				if (
					Math.abs(targetPose.getX() - currentPose.getX()) <
					stage.getXThreshold() &&
					Math.abs(targetPose.getY() - currentPose.getY()) <
					stage.getYThreshold() &&
					Math.abs(
						targetPose.getRotation().getDegrees() -
						currentPose.getRotation().getDegrees()
					) <
					stage.getRotationalThreshold()
				) {
					timesCorrect++;
				} else {
					timesCorrect = 0;
				}

				break;
			case FOLLOW_POSE_2D_FIELD:
				targetPose = stage.pose;
				currentPose = swerve.getRobotPose();

				xSpeed =
					ExtendedMath.clamp(
						-stage.speed,
						stage.speed,
						translationPID.calculate(
							currentPose.getTranslation().getX(),
							targetPose.getTranslation().getX()
						)
					);
				ySpeed =
					ExtendedMath.clamp(
						-stage.speed,
						stage.speed,
						translationPID.calculate(
							currentPose.getTranslation().getY(),
							targetPose.getTranslation().getY()
						)
					);
				rotationalSpeed =
					ExtendedMath.clamp(
						-stage.speed,
						stage.speed,
						rotationPID.calculate(
							currentPose.getRotation().getRadians(),
							targetPose.getRotation().getRadians()
						)
					);

				swerve.driveFieldCentric(
					xSpeed / 2,
					ySpeed / 2,
					rotationalSpeed / 3
				);

				if (
					Math.abs(targetPose.getX() - currentPose.getX()) <
					stage.getXThreshold() &&
					Math.abs(targetPose.getY() - currentPose.getY()) <
					stage.getYThreshold() &&
					Math.abs(
						targetPose.getRotation().getDegrees() -
						currentPose.getRotation().getDegrees()
					) <
					stage.getRotationalThreshold()
				) {
					timesCorrect++;
				} else {
					timesCorrect = 0;
				}

				break;
			default:
				break;
		}
		if (timesCorrect >= stage.getTimeThreshold() * 50) {
			currentStage++;
			if (currentStage >= stages.length) {
				finished = true;
			} else {
				stage = stages[currentStage];
			}
			timesCorrect = 0;
		}
	}

	@Override
	public boolean isFinished() {
		return finished;
	}

	@Override
	public void end(boolean interrupted) {
		swerve.driveRobotCentric(0, 0, 0);
	}

	public enum StageType {
		VISION_ALIGN_TARGET_ROTATION,
		VISION_ALIGN_TARGET_TRANSLATION,
		FOLLOW_POSE_2D_RELATIVE,
		FOLLOW_POSE_2D_FIELD,
	}

	public static class AutonomousDriveStage {

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

		public int getLimelightId() {
			return limelightId;
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

		public double getRotationalThreshold() {
			return rotationalThreshold;
		}

		public int getTimeThreshold() {
			return timeThreshold;
		}

		public double getSpeed() {
			return speed;
		}
	}
}
