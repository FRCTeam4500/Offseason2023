package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.EnumConstants.TalonType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.SparkMaxMotorController;
import frc.robot.hardware.TalonMotorController;
import frc.robot.hardware.interfaces.SwerveMotorController;
import frc.robot.utilities.Conversions;

public class SwerveModule {

	private TalonMotorController driveMotor;
	private TalonMotorController angleMotor;
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
		driveMotor = new TalonMotorController(driveId, TalonType.TalonFX);

		angleMotor = new TalonMotorController(angleId, TalonType.TalonFX);

		driveMotor.configureForSwerve(invertDrive, 35, drivekP, 0, true);
		angleMotor.configureForSwerve(invertAngle, 25, anglekP, 0, false);
		this.translationFromCenter = translationToCenter;
	}

	public void drive(SwerveModuleState initialTargetState) {
		SwerveModuleState targetState = TalonOptimizer.optimize(
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

	/*
	 * EXPERIMENTAL START
	 */

	/** TODO: Find Feedforward Values */
	public static final double driveKS = (0.32 / 12);
	public static final double driveKV = (1.51 / 12);
	public static final double driveKA = (0.27 / 12);

	SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
		driveKS,
		driveKV,
		driveKA
	);

	public void setDesiredState(
		SwerveModuleState desiredState,
		boolean isOpenLoop
	) {
		/* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
		desiredState =
			TalonOptimizer.optimize(desiredState, getModuleState().angle);
		setAngle(desiredState);
		setSpeed(desiredState, isOpenLoop);
	}

	private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
		if (isOpenLoop) {
			double percentOutput =
				desiredState.speedMetersPerSecond /
				SwerveConstants.MAX_LINEAR_SPEED;
			driveMotor.set(ControlMode.PercentOutput, percentOutput);
		} else {
			double velocity = Conversions.MPSToFalcon(
				desiredState.speedMetersPerSecond,
				SwerveConstants.WHEEL_DIAMETER * Math.PI,
				SwerveConstants.DRIVE_RATIO
			);
			driveMotor.set(
				ControlMode.Velocity,
				velocity,
				DemandType.ArbitraryFeedForward,
				feedforward.calculate(desiredState.speedMetersPerSecond)
			);
		}
	}

	private void setAngle(SwerveModuleState desiredState) {
		// Prevent rotating module if speed is less then 1%. Prevents Jittering.
		Rotation2d angle = (
				Math.abs(desiredState.speedMetersPerSecond) <=
				(SwerveConstants.MAX_LINEAR_SPEED * 0.01)
			)
			? new Rotation2d(angleMotor.getAngle())
			: desiredState.angle;

		angleMotor.set(
			ControlMode.Position,
			Conversions.degreesToFalcon(
				angle.getDegrees(),
				SwerveConstants.ANGLE_RATIO
			)
		);
	}

	/*
	 * EXPERIMENTAL END
	 */

	static class TalonOptimizer {

		/**
		 * Minimize the change in heading the desired swerve module state would require by potentially
		 * reversing the direction the wheel spins. Customized from WPILib's version to include placing
		 * in appropriate scope for CTRE onboard control.
		 *
		 * @param desiredState The desired state.
		 * @param currentAngle The current module angle.
		 */
		public static SwerveModuleState optimize(
			SwerveModuleState desiredState,
			Rotation2d currentAngle
		) {
			double targetAngle = placeInAppropriate0To360Scope(
				currentAngle.getDegrees(),
				desiredState.angle.getDegrees()
			);
			double targetSpeed = desiredState.speedMetersPerSecond;
			double delta = targetAngle - currentAngle.getDegrees();
			if (Math.abs(delta) > 90) {
				targetSpeed = -targetSpeed;
				targetAngle =
					delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
			}
			return new SwerveModuleState(
				targetSpeed,
				Rotation2d.fromDegrees(targetAngle)
			);
		}

		/**
		 * @param scopeReference Current Angle
		 * @param newAngle Target Angle
		 * @return Closest angle within scope
		 */
		private static double placeInAppropriate0To360Scope(
			double scopeReference,
			double newAngle
		) {
			double lowerBound;
			double upperBound;
			double lowerOffset = scopeReference % 360;
			if (lowerOffset >= 0) {
				lowerBound = scopeReference - lowerOffset;
				upperBound = scopeReference + (360 - lowerOffset);
			} else {
				upperBound = scopeReference - lowerOffset;
				lowerBound = scopeReference - (360 + lowerOffset);
			}
			while (newAngle < lowerBound) {
				newAngle += 360;
			}
			while (newAngle > upperBound) {
				newAngle -= 360;
			}
			if (newAngle - scopeReference > 180) {
				newAngle -= 360;
			} else if (newAngle - scopeReference < -180) {
				newAngle += 360;
			}
			return newAngle;
		}
	}

	public SwerveMotorController getDriveMotor() {
		return driveMotor;
	}

	public SwerveMotorController getAngleMotor() {
		return angleMotor;
	}

	public void setTranslationFromCenter(Translation2d translationFromCenter) {
		this.translationFromCenter = translationFromCenter;
	}
}
