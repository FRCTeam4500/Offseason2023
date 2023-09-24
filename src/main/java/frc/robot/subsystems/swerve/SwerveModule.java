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

/** A class that represents one swerve module on the robot, containing a drive motor and a angle motor */
public class SwerveModule {

	/** The drive motor of the module. Used to set the velocity of the module */
	private SwerveMotorController driveMotor;
	/** The angle motor of the module. Used to set the angle of the module */
	private SwerveMotorController angleMotor;
	/** The translation of the swerve module from the center of the robot. Used during kinematics operations */
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

	/**
	 * Takes in a target state of the module, and sets the motors to meet that state. Should only be called by the drive methods of the SwerveDrive Class
	 * <p><strong>Note: </strong> the target velocity is multiplied by the cosine of the distance to the target angle.
	 * This means that while the wheel is rotating to its target angle, it will be scaled down proportionally to how far off from the target angle it is
	 * @param initialTargetState the target state this module should reach
	 */
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

	/**
	 * Gets the current state of the module
	 * @return a SwerveModuleState object containing the module's velocity and angle
	 */
	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(
			driveMotor.getAngularVelocity() *
			SwerveConstants.DRIVE_RATIO *
			SwerveConstants.WHEEL_DIAMETER /
			2,
			new Rotation2d(angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO)
		);
	}

	/**
	 * Gets the current velocity of the angle motor or the specific module
	 * @return the current velocity of the angle motor in m/s
	 */
	public double getAngularVelocity() {
		return angleMotor.getAngularVelocity() * SwerveConstants.ANGLE_RATIO;
	}

	/**
	 * Gets the current position of the module
	 * @return a SwerveModulePosition containing the drive wheel's distance in meters and the angle of the module in radians
	 */
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

	/**
	 * Gets the translation of this module from the center of the robot
	 * @return the translation of the module
	 */
	public Translation2d getTranslationFromCenter() {
		return translationFromCenter;
	}

	/**
	 * Directly sets the angle of the module
	 * @param targetAngle the new target angle of the module in radians
	 */
	public void setModuleAngle(double targetAngle) {
		angleMotor.setAngle(targetAngle / SwerveConstants.ANGLE_RATIO);
	}

	/**
	 * Directly sets the velocity of the module
	 * @param targetVelocity the new target velocity of the module in m/s
	 */
	public void setModuleVelocity(double targetVelocity) {
		driveMotor.setAngularVelocity(
			targetVelocity *
			2 /
			(SwerveConstants.DRIVE_RATIO * SwerveConstants.WHEEL_DIAMETER)
		);
	}
}
