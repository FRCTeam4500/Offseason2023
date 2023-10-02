package frc.robot.subsystems.swervydwervy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class Odematics {

	private final SwerveDriveOdometry odometry;

	public Odematics(
		SwerveDriveKinematics kinematics,
		Pose2d initialPose,
		Rotation2d gyroAngle,
		SwerveModulePosition... swerveModulePositions
	) {
		this.odometry =
			new SwerveDriveOdometry(
				kinematics,
				gyroAngle,
				swerveModulePositions,
				initialPose
			);
	}

	public Odematics(
		SwerveDriveKinematics kinematics,
		Rotation2d gyroAngle,
		SwerveModulePosition... swerveModulePositions
	) {
		this(kinematics, new Pose2d(), gyroAngle, swerveModulePositions);
	}

	public Pose2d getPoseMeters() {
		return odometry.getPoseMeters();
	}

	public void resetPosition(
		Pose2d pose,
		Rotation2d gyroAngle,
		SwerveModulePosition... swerveModulePositions
	) {
		odometry.resetPosition(gyroAngle, swerveModulePositions, pose);
	}

	public Pose2d update(
		Rotation2d gyroAngle,
		SwerveModulePosition... modulePositions
	) {
		return odometry.update(gyroAngle, modulePositions);
	}
}
