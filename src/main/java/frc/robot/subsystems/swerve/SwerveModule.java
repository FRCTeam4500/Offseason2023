package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.SparkMaxMotorController;
import frc.robot.hardware.TalonMotorController;
import frc.robot.hardware.interfaces.SwerveMotorController;

public class SwerveModule {
	private SwerveMotorController driveMotor;
	private SwerveMotorController angleMotor;
	private Translation2d translationFromCenter;

	/**
	 * Creates a new swerve module
	 * @param driveId The CAN ID of the drive motor
	 * @param angleId The CAN ID of the angle motor
	 * @param translationToCenter The translation of the swerve module relative to the center of the robot
	 * @param invertDrive Whether to invert the direction of the drive motor
	 * @param invertAngle Whether to invert the direction of the angle motor
	 * @param drivekP The P value used in the PID controller of the drive motor
	 * @param anglekP The P value used in the PID controller of the angle motor
	 */
	public SwerveModule(
		int driveId,
		int angleId,
		Translation2d translationToCenter,
		boolean invertDrive,
		boolean invertAngle,
		double drivekP,
		double anglekP,
		boolean neoDrive,
		boolean neoAngle
	) {
		if (neoDrive) {
			driveMotor = new SparkMaxMotorController(driveId, MotorType.kBrushless);
		} else {
			driveMotor = new TalonMotorController(driveId, "Talon FX");
		}

		if (neoAngle) {
			angleMotor = new SparkMaxMotorController(angleId, MotorType.kBrushless);
		} else {
			angleMotor = new TalonMotorController(angleId, "Talon FX");
		}

		driveMotor.configureForSwerve(invertDrive, 35, drivekP, 0, true);
		angleMotor.configureForSwerve(invertAngle, 25, anglekP, 0, false);
		this.translationFromCenter = translationToCenter;
	}

	public void drive(SwerveModuleState initialTargetState) {
		SwerveModuleState targetState = SwerveModuleState.optimize(
			initialTargetState,
			getModuleState().angle
		);
		setModuleVelocity(
			targetState.speedMetersPerSecond * // This is multiplying the drive wheel's velocity by the cosine of how far the modules angle is from where it should be.
			Math.abs(targetState.angle.minus(getModuleState().angle).getCos())
		);
		setModuleAngle(targetState.angle.getRadians());
	}

	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(
			driveMotor.getAngularVelocity() *
			SwerveConstants.DRIVE_RATIO *
			SwerveConstants.WHEEL_DIAMETER /
			2,
			new Rotation2d(angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO)
		);
	}

	public double getAngularVelocity() {
		return angleMotor.getAngularVelocity() * SwerveConstants.ANGLE_RATIO;
	}

	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(
			driveMotor.getAngle() /
			(2 * Math.PI) * 
			SwerveConstants.DRIVE_RATIO *
			SwerveConstants.WHEEL_DIAMETER *
			Math.PI,
			getModuleState().angle
		);
	}

	public Translation2d getTranslationFromCenter() {
		return translationFromCenter;
	}

	public void setModuleAngle(double targetAngleRadians) {
		angleMotor.setAngle(targetAngleRadians / SwerveConstants.ANGLE_RATIO);
	}

	public void setModuleVelocity(double targetVelocityMetersPerSecond) {
		driveMotor.setAngularVelocity(
			targetVelocityMetersPerSecond *
			2 /
			(SwerveConstants.DRIVE_RATIO * SwerveConstants.WHEEL_DIAMETER)
		);
	}
}
