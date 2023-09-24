package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.Gyro;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.vision.Vision;

/**
 * Subsystem class which represents the drivetrain of our robot
 * <p> Used to move the robot chassis
 */
public class SwerveDrive extends SubsystemBase implements SwerveDriveInterface {
	private Gyro gyro;	
	private SwerveModule[] modules;
	private SwerveDriveKinematics kinematics;
	private SwerveDrivePoseEstimator poseEstimator;
	private static SwerveDrive instanceSwerve;
	private Vision vision;
	private double currentGyroZero;
	private DriveInputsAutoLogged inputs = new DriveInputsAutoLogged();

	public DriveInputsAutoLogged getInputs() {
		return inputs;
	}

	private SwerveDrive() {
		SwerveModule[] modules = {
			new SwerveModule(
				SwerveConstants.DFLPORT,
				SwerveConstants.AFLPORT,
				SwerveConstants.FRONT_LEFT_MODULE_TRANSLATION,
				true,
				false,
				0.1,
				0.3,
				false,
				false
			),
			new SwerveModule(
				SwerveConstants.DFRPORT,
				SwerveConstants.AFRPORT,
				SwerveConstants.FRONT_RIGHT_MODULE_TRANSLATION,
				false,
				false,
				0.1,
				0.3,
				false,
				false
			),
			new SwerveModule(
				SwerveConstants.DBLPORT,
				SwerveConstants.ABLPORT,
				SwerveConstants.BACK_LEFT_MODULE_TRANSLATION,
				true,
				false,
				0.1,
				0.3,
				false,
				false
			),
			new SwerveModule(
				SwerveConstants.DBRPORT,
				SwerveConstants.ABRPORT,
				SwerveConstants.BACK_RIGHT_MODULE_TRANSLATION,
				false,
				false,
				0.1,
				0.3,
				false,
				false
			),
		};
		this.modules = modules;
		currentGyroZero = 0;
		gyro = new Gyro(I2C.Port.kMXP);
		kinematics = new SwerveDriveKinematics(getModuleTranslations());
		poseEstimator =
			new SwerveDrivePoseEstimator(
				kinematics,
				new Rotation2d(gyro.getAngle()),
				getModulePositions(),
				new Pose2d()
			);
		this.vision = Vision.getInstance();
	}

	/**
	 * Gets the instance of the swerve drive. If the instance doesn't exist, it creates it
	 * @return the instance of the swerve drive
	 */
	public static synchronized SwerveDrive getInstance() {
		if (instanceSwerve == null) {
			instanceSwerve = new SwerveDrive();
		}
		return instanceSwerve;
	}

	/**
	 * A method from Subsystem Base. <p>Our code probably shouldn't call this. This is called every scheduler tick (20ms) by WPI. Code that needs to be run repeadly should be put here
	 */
	@Override
	public void periodic() {
		poseEstimator.update(
			new Rotation2d(gyro.getAngle()),
			getModulePositions()
		);
		if (vision.hasValidTargets(0)) {
			poseEstimator.addVisionMeasurement(
				vision.getRobotPose(0),
				Timer.getFPGATimestamp()
			);
		}
	}

	/**
	 * Drives the robot relative to the field (forward is away from the alliance wall) based on target forward, sideways, and rotational velocities
	 * @param forwardVelocity the target forward velocity of the robot. Units are m/s and forward is positive
	 * @param sidewaysVelocity the target sideways velocity of the robot. Units are m/s and left is positive
	 * @param rotationalVelocity the target rotational velocity of the robot. Units are rad/s and counter-clockwise is positive
	 */
	public void driveFieldCentric(
		double forwardVelocity,
		double sidewaysVelocity,
		double rotationalVelocity
	) {
		driveModules(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				forwardVelocity,
				sidewaysVelocity,
				rotationalVelocity,
				new Rotation2d(getRobotAngle())
			)
		);
	}

	/**
	 * Drives the robot relative to itself based on target forward, sideways, and rotational velocities
	 * @param forwardVelocity the target forward velocity of the robot. Units are m/s and forward is positive
	 * @param sidewaysVelocity the target sideways velocity of the robot. Units are m/s and left is positive
	 * @param rotationalVelocity the target rotational velocity of the robot. Units are rad/s and counter-clockwise is positive
	 */
	public void driveRobotCentric(
		double forwardVelocity,
		double sidewaysVelocity,
		double rotationalVelocity
	) {
		driveModules(
			new ChassisSpeeds(
				forwardVelocity,
				sidewaysVelocity,
				rotationalVelocity
			)
		);
	}

	/**
	 * Uses inverse kinematics to convert a robot-relative set of chassis speeds into states to send to the swerve modules
	 * @param targetChassisSpeeds The target robot-relative chassis speeds
	 */
	public void driveModules(ChassisSpeeds targetChassisSpeeds) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(
			discretize(targetChassisSpeeds)
		);
		SwerveDriveKinematics.desaturateWheelSpeeds(
			states,
			SwerveConstants.MAX_LINEAR_SPEED
		);
		for (int i = 0; i < modules.length; i++) {
			modules[i].drive(states[i]);
		}
	}

	/** 
	 * Fixes situation where robot drifts in the direction it's rotating in if turning and translating at the same time 
	 * @see https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
	*/
	private static ChassisSpeeds discretize(ChassisSpeeds originalChassisSpeeds) {
		double vx = originalChassisSpeeds.vxMetersPerSecond;
		double vy = originalChassisSpeeds.vyMetersPerSecond;
		double omega = originalChassisSpeeds.omegaRadiansPerSecond;
		double dt = 0.02; // This should be the time these values will be used, so normally just the loop time
		Pose2d desiredDeltaPose = new Pose2d(
			vx * dt,
			vy * dt,
			new Rotation2d(omega * dt)
		);
		Twist2d twist = new Pose2d().log(desiredDeltaPose);
		return new ChassisSpeeds(
			twist.dx / dt,
			twist.dy / dt,
			twist.dtheta / dt
		);
	}

	public Gyro getGyro() {
		return gyro;
	}

	public void zeroModules() {
		for (SwerveModule module : modules) {
			module.setModuleVelocity(0);
			module.setModuleAngle(0);
		}
	}

	/**
	 * Gets the translations of the swerve modules from the center of the robot. Used when initializing the kinematics object
	 * @return an array containing the translations of the swerve modules in the same order they were put into the SwerveDrive constructor
	 */
	public Translation2d[] getModuleTranslations() {
		Translation2d[] translations = new Translation2d[modules.length];
		for (int i = 0; i < modules.length; i++) {
			translations[i] = modules[i].getTranslationFromCenter();
		}
		return translations;
	}

	/**
	 * Gets the states of the swerve modules. Used primarily for kinematics
	 * @return an array containing the states of the swerve modules in the same order they were put into the SwerveDrive constructor
	 */
	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++) {
			states[i] = modules[i].getModuleState();
		}
		return states;
	}

	/**
	 * Gets the positions of the swerve modules. Used primarily for poseEstimator
	 * @return an array containing the positions of the swerve modules in the same order they were put into the SwerveDrive constructor
	 */
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++) {
			positions[i] = modules[i].getModulePosition();
		}
		return positions;
	}

	/**
	 * Uses forward kinematics to turn the swerve module's states into a chassis speeds for the robot
	 * @return The robot's current chassis speeds
	 */
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	/**
	 * Gets the robot's current odometric position -- Estimated Using Vision if Possible
	 * @return the robot's current pose
	 */
	public Pose2d getRobotPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/**
	 * Sets the robot's odometric position to a given position
	 * @param newPose the pose the robot should be
	 */
	public void resetPose(Pose2d newPose) {
		poseEstimator.resetPosition(
			new Rotation2d(gyro.getAngle()),
			getModulePositions(),
			newPose
		);
	}

	/**
	 * Gets the angle of the robot relative to the current field-centric zero
	 * @return the robot's field-centric angle in radians
	 */
	public double getRobotAngle() {
		return gyro.getAngle() - currentGyroZero;
	}

	/**
	 * Sets the field-centric zero to some angle relative to the robot
	 * <p>CCW is positive
	 * @param offset the angle relative to the robot, in radians
	 */
	public void resetRobotAngle(double offset) {
		currentGyroZero = gyro.getAngle() - offset;
		MessagingSystem.getInstance().addMessage("Swerve -> Reset Gyro");
	}

	/**
	 * Sets where the robot is currently facing to the field-centric zero
	 */
	public void resetRobotAngle() {
		resetRobotAngle(0);
	}

	public double getCurrentZero() {
		return currentGyroZero;
	}

	/**
	 * Update with real values
	 * <p>Order:
	 * <p>Front Left, Front Right, Back Left, Back Right
	 */
	@Override
	public void updateInputs(DriveInputs inputs) {
		// inputs.frontLeftModuleDriveMeters =
		// 	modules[0].getModulePosition().distanceMeters;
		inputs.frontLeftModuleDriveVelocity =
			modules[0].getModuleState().speedMetersPerSecond;
		inputs.frontLeftModuleAngleRad =
			modules[0].getModuleState().angle.getRadians();
		// inputs.frontLeftModuleAngleVelocity = modules[0].getAngularVelocity();

		// inputs.frontRightModuleDriveMeters =
		// 	modules[1].getModulePosition().distanceMeters;
		inputs.frontRightModuleDriveVelocity =
			modules[1].getModuleState().speedMetersPerSecond;
		inputs.frontRightModuleAngleRad =
			modules[1].getModuleState().angle.getRadians();
		// inputs.frontRightModuleAngleVelocity = modules[1].getAngularVelocity();

		// inputs.backLeftModuleDriveMeters =
		// 	modules[2].getModulePosition().distanceMeters;
		inputs.backLeftModuleDriveVelocity =
			modules[2].getModuleState().speedMetersPerSecond;
		inputs.backLeftModuleAngleRad =
			modules[2].getModuleState().angle.getRadians();
		// inputs.backLeftModuleAngleVelocity = modules[2].getAngularVelocity();

		// inputs.backRightModuleDriveMeters =
		// 	modules[3].getModulePosition().distanceMeters;
		inputs.backRightModuleDriveVelocity =
			modules[3].getModuleState().speedMetersPerSecond;
		inputs.backRightModuleAngleRad =
			modules[3].getModuleState().angle.getRadians();
		// inputs.backRightModuleAngleVelocity = modules[3].getAngularVelocity();

		// inputs.gyroYawRad = gyro.getAngle();
		// inputs.gyroPitchRad = gyro.getPitch();
		// inputs.gyroRollRad = gyro.getRoll();
	}

	/**
	 * A method from SubsystemBase. <p>Adds properties of the subsystem to shuffleboard. Our code should never call this class, that is done by WPI
	 */
	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Gyro Angle: ", () -> gyro.getAngle(), null);
		builder.addDoubleProperty(
			"Gyro Offset From Zero: ",
			() -> getRobotAngle() % (2 * Math.PI),
			null
		);
		builder.addDoubleProperty(
			"Current Forward Speed: ",
			() -> getChassisSpeeds().vxMetersPerSecond,
			null
		);
		builder.addDoubleProperty(
			"Current Sideways Speed: ",
			() -> getChassisSpeeds().vyMetersPerSecond,
			null
		);
		builder.addDoubleProperty(
			"Current Rotational Speed: ",
			() -> getChassisSpeeds().omegaRadiansPerSecond,
			null
		);
		builder.addDoubleProperty(
			"Odometric X: ",
			() -> getRobotPose().getX(),
			null
		);
		builder.addDoubleProperty(
			"Odometric Y: ",
			() -> getRobotPose().getY(),
			null
		);
		builder.addDoubleProperty(
			"Odometric Rotation: ",
			() -> getRobotPose().getRotation().getRadians(),
			null
		);
	}
}
