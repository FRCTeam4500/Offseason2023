package frc.robot.subsystems.swervydwervy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnumConstants.TalonType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.Gyro;
import frc.robot.hardware.TalonMotorController;
import frc.robot.subsystems.swervydwervy.Estimator;
import frc.robot.subsystems.vision.Vision;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;

public class Swerve extends SubsystemBase implements SwerveInterface {

	private final SwerveModule[] swerveModules;
	private final SwerveDriveKinematics kinematics;
	private Estimator odometry;
	private final Gyro gyro;
	private final double maxSpeedMetersPerSecond;
	private Rotation2d gyroOffset = new Rotation2d();
	private boolean hasGyroOffset = false;
	private DriveInputsAutoLogged inputs = new DriveInputsAutoLogged();
	private Vision vision = Vision.getInstance();

	public DriveInputsAutoLogged getInputs() {
		return inputs;
	}

	private static Swerve instance = null;

	/**
	 * Construct a swerve drive object. Along with a gyro, this takes in four
	 * configured swerve
	 * modules, by convention in left front, right front, left rear, right rear
	 * order.
	 *
	 * @param gyro          the gyro to use for field-centric driving
	 * @param swerveModules the swerve modules
	 */
	private Swerve(Gyro gyro, SwerveModule... swerveModules) {
		this.gyro = gyro;
		this.swerveModules = swerveModules;

		/*
		 * final List<Translation2d> locations = Arrays.stream(swerveModules)
		 * .map(SwerveModule::getWheelLocationMeters)
		 * .collect(Collectors.toList());
		 */

		Translation2d[] translation2ds = Arrays
			.stream(swerveModules)
			.map(SwerveModule::getWheelLocationMeters)
			.toArray(Translation2d[]::new);

		// verify all swerve modules are set to same max speed
		Set<Double> maxSpeeds = Arrays
			.stream(swerveModules)
			.map(SwerveModule::getMaxSpeedMetersPerSecond)
			.collect(Collectors.toSet());

		if (maxSpeeds.size() > 1) {
			throw new IllegalStateException(
				"swerve modules must have same driveMaximumMetersPerSecond"
			);
		}
		maxSpeedMetersPerSecond = swerveModules[0].getMaxSpeedMetersPerSecond();

		SwerveModulePosition[] modulePositions = Arrays
			.stream(swerveModules)
			.map(SwerveModule::getPosition)
			.toArray(SwerveModulePosition[]::new);

		kinematics = new SwerveDriveKinematics(translation2ds);
		odometry =
			new Estimator(
				gyro.getRotation2d(),
				new Pose2d(),
				kinematics,
				new Matrix<>(VecBuilder.fill(2, 2, Math.PI)),
				new Matrix<>(VecBuilder.fill(0.5)),
				new Matrix<>(VecBuilder.fill(2, 2, Math.PI)),
				modulePositions
			);
	}

	/**
	 * Construct a swerve drive object with a navX gyro. This takes in four
	 * configured swerve modules,
	 * by convention in left front, right front, left rear, right rear order.
	 *
	 * @param swerveModules the swerve modules
	 */
	private Swerve(SwerveModule... swerveModules) {
		this(new Gyro(Port.kMXP), swerveModules);
	}

	public static synchronized Swerve getInstance() {
		if (instance == null) {
			instance =
				new Swerve(
					new SwerveModule(
						new TalonMotorController(
							SwerveConstants.DFLPORT,
							TalonType.TalonFX
						),
						new TalonMotorController(
							SwerveConstants.AFLPORT,
							TalonType.TalonFX
						),
						SwerveConstants.FRONT_LEFT_MODULE_TRANSLATION
					),
					new SwerveModule(
						new TalonMotorController(
							SwerveConstants.DFRPORT,
							TalonType.TalonFX
						),
						new TalonMotorController(
							SwerveConstants.AFRPORT,
							TalonType.TalonFX
						),
						SwerveConstants.FRONT_RIGHT_MODULE_TRANSLATION
					),
					new SwerveModule(
						new TalonMotorController(
							SwerveConstants.DBLPORT,
							TalonType.TalonFX
						),
						new TalonMotorController(
							SwerveConstants.ABLPORT,
							TalonType.TalonFX
						),
						SwerveConstants.BACK_LEFT_MODULE_TRANSLATION
					),
					new SwerveModule(
						new TalonMotorController(
							SwerveConstants.DBRPORT,
							TalonType.TalonFX
						),
						new TalonMotorController(
							SwerveConstants.ABRPORT,
							TalonType.TalonFX
						),
						SwerveConstants.BACK_RIGHT_MODULE_TRANSLATION
					)
				);
		}
		return instance;
	}

	/**
	 * Update with real values
	 * <p>
	 * Order:
	 * <p>
	 * Front Left, Front Right, Back Left, Back Right
	 */
	@Override
	public void updateInputs(DriveInputs inputs) {
		inputs.frontLeftModuleDriveVelocity =
			swerveModules[0].getState().speedMetersPerSecond;
		inputs.frontLeftAngleRadians =
			swerveModules[0].getState().angle.getRadians();
		inputs.frontRightModuleDriveVelocity =
			swerveModules[1].getState().speedMetersPerSecond;
		inputs.frontRightAngleRadians =
			swerveModules[1].getState().angle.getRadians();
		inputs.backLeftModuleDriveVelocity =
			swerveModules[2].getState().speedMetersPerSecond;
		inputs.backLeftAngleRadians =
			swerveModules[2].getState().angle.getRadians();
		inputs.backRightModuleDriveVelocity =
			swerveModules[3].getState().speedMetersPerSecond;
		inputs.backRightAngleRadians =
			swerveModules[3].getState().angle.getRadians();
	}

	/**
	 * Replace the default {@code Estimator} with another such as
	 * {@code
	 * Odematics}. This is provided as a setter so that the
	 * current configured
	 * {@code SwerveDriveKinematics} can be used in constructing the
	 * {@code Estimator}.
	 *
	 * <p>
	 * <strong>IMPORTANT:</strong> This should be called right after the constructor
	 * and before any
	 * use of the drive.
	 *
	 * @param odometry the replacement odometry calculation method
	 */
	public void setOdometry(Estimator odometry) {
		this.odometry = odometry;
	}

	/**
	 * Returns the kinematics object in use by this swerve drive.
	 *
	 * @return The kinematics object in use.
	 */
	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	/**
	 * Returns the position of the robot on the field.
	 *
	 * @return the pose of the robot (x and y are in meters)
	 */
	public Pose2d getPoseMeters() {
		return odometry.getPoseMeters();
	}

	/**
	 * Returns the current gyro-measured heading of the robot. This will be affected
	 * by any gyro drift
	 * that may have accumulated since last gyro recalibration. The angle is
	 * continuous, that is it
	 * will continue from 360 to 361 degrees. This allows algorithms that wouldn't
	 * want to see a
	 * discontinuity in the gyro output as it sweeps past from 360 to 0 on the
	 * second time around. The
	 * angle is expected to increase as the gyro turns counterclockwise when looked
	 * at from the top.
	 *
	 * @return the Rotation2d of the robot relative to gyro zero
	 */
	public Rotation2d getHeading() {
		return hasGyroOffset
			? gyro.getRotation2d().rotateBy(gyroOffset)
			: gyro.getRotation2d();
	}

	/**
	 * Returns the current gyro-measured heading of the robot. This will be affected
	 * by any gyro drift
	 * that may have accumulated since last gyro recalibration. The angle is
	 * continuous, that is it
	 * will continue from 360 to 361 degrees. This allows algorithms that wouldn't
	 * want to see a
	 * discontinuity in the gyro output as it sweeps past from 360 to 0 on the
	 * second time around. The
	 * angle is expected to increase as the gyro turns clockwise when looked at from
	 * the top.
	 *
	 * @return the current heading in degrees of the robot relative to gyro zero
	 */
	double getGyroAngle() {
		// FIXME: does not have gyro offset
		return gyro.getAngle();
	}

	/**
	 * Return the rate of rotation of the gyro. The rate is based on the most recent
	 * reading of the
	 * gyro analog value. The rate is expected to be positive as the gyro turns
	 * clockwise when looked
	 * at from the top.
	 *
	 * @return the current rate in degrees per second
	 */
	public double getGyroRate() {
		return gyro.getRate();
	}

	/**
	 * Get the current gyro offset applied to the IMU gyro angle during field
	 * oriented driving.
	 *
	 * @return the gyro offset
	 */
	public Rotation2d getGyroOffset() {
		return gyroOffset;
	}

	/**
	 * Set the current gyro offset applied to the IMU gyro angle during field
	 * oriented driving,
	 * defaults to zero.
	 *
	 * @param gyroOffset the desired offset
	 */
	public void setGyroOffset(Rotation2d gyroOffset) {
		if (this.gyroOffset.equals(gyroOffset)) {
			return;
		}
		this.gyroOffset = gyroOffset;
		hasGyroOffset = true;
	}

	/**
	 * Get the configured swerve modules.
	 *
	 * @return array of swerve modules
	 */
	public SwerveModule[] getSwerveModules() {
		return swerveModules;
	}

	/**
	 * Resets the robot's position on the field. Any accumulated gyro drift will be
	 * noted and
	 * accounted for in subsequent calls to {@link #getPoseMeters()}.
	 *
	 * @param pose The robot's actual position on the field.
	 */
	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(
			pose,
			gyro.getRotation2d().rotateBy(gyroOffset),
			swerveModules[0].getPosition(),
			swerveModules[1].getPosition(),
			swerveModules[2].getPosition(),
			swerveModules[3].getPosition()
		);
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetDriveEncoders() {
		for (int i = 0; i < 4; i++) {
			swerveModules[i].resetDriveEncoder();
		}
	}

	/** Resets the gyro to a heading of zero. */
	public void resetGyro() {
		gyro.reset();
	}

	/**
	 * Update the swerve drive odometry state. Call this from the drive subsystem
	 * {@code periodic()}
	 * method.
	 */
	public void periodic() {
		odometry.update(
			hasGyroOffset
				? gyro.getRotation2d().rotateBy(gyroOffset)
				: gyro.getRotation2d(),
			swerveModules[0].getPosition(),
			swerveModules[1].getPosition(),
			swerveModules[2].getPosition(),
			swerveModules[3].getPosition()
		);
		if (vision.hasValidTargets(0)) {
			odometry.addVisionMeasurement(
				vision.getRobotPose(0),
				Timer.getFPGATimestamp()
			);
		}
	}

	/**
	 * Drive the robot with given x, y, and rotational velocities using open-loop
	 * velocity control.
	 *
	 * @param vxMetersPerSecond     the desired x velocity component
	 * @param vyMetersPerSecond     the desired y velocity component
	 * @param omegaRadiansPerSecond the desired rotational velocity component
	 * @param isFieldOriented       true if driving field-oriented
	 */
	public void drive(
		double vxMetersPerSecond,
		double vyMetersPerSecond,
		double omegaRadiansPerSecond,
		boolean isFieldOriented
	) {
		SwerveModuleState[] swerveModuleStates = getSwerveModuleStates(
			vxMetersPerSecond,
			vyMetersPerSecond,
			omegaRadiansPerSecond,
			isFieldOriented
		);
		for (int i = 0; i < 4; i++) {
			swerveModules[i].setDesiredState(swerveModuleStates[i], true);
		}
	}

	/**
	 * Drive the robot with given chassis speeds using open-loop
	 * velocity control.
	 *
	 * @param chassisSpeeds speeds the desired chassis speeds
	 */
	public void drive(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] swerveModuleStates = getSwerveModuleStates(
			chassisSpeeds.vxMetersPerSecond,
			chassisSpeeds.vyMetersPerSecond,
			chassisSpeeds.omegaRadiansPerSecond,
			true
		);
		for (int i = 0; i < 4; i++) {
			swerveModules[i].setDesiredState(swerveModuleStates[i], true);
		}
	}

	/**
	 * Move the robot with given x, y, and rotational velocities using closed-loop
	 * velocity control.
	 *
	 * @param vxMetersPerSecond     the desired x velocity component
	 * @param vyMetersPerSecond     the desired y velocity component
	 * @param omegaRadiansPerSecond the desired rotational velocity component
	 * @param isFieldOriented       true if driving field-oriented
	 */
	public void move(
		double vxMetersPerSecond,
		double vyMetersPerSecond,
		double omegaRadiansPerSecond,
		boolean isFieldOriented
	) {
		SwerveModuleState[] swerveModuleStates = getSwerveModuleStates(
			vxMetersPerSecond,
			vyMetersPerSecond,
			omegaRadiansPerSecond,
			isFieldOriented
		);
		for (int i = 0; i < 4; i++) {
			swerveModules[i].setDesiredState(swerveModuleStates[i], false);
		}
	}

	/**
	 * Module states as array. Used for Logger.
	 *
	 * @param vxMetersPerSecond
	 * @param vyMetersPerSecond
	 * @param omegaRadiansPerSecond
	 * @param isFieldOriented
	 * @return
	 */
	private SwerveModuleState[] getSwerveModuleStates(
		double vxMetersPerSecond,
		double vyMetersPerSecond,
		double omegaRadiansPerSecond,
		boolean isFieldOriented
	) {
		ChassisSpeeds chassisSpeeds = isFieldOriented
			? ChassisSpeeds.fromFieldRelativeSpeeds(
				vxMetersPerSecond,
				vyMetersPerSecond,
				omegaRadiansPerSecond,
				hasGyroOffset
					? gyro.getRotation2d().rotateBy(gyroOffset)
					: gyro.getRotation2d()
			)
			: new ChassisSpeeds(
				vxMetersPerSecond,
				vyMetersPerSecond,
				omegaRadiansPerSecond
			);

		ChassisSpeeds discretizedSpeeds = discretize(chassisSpeeds);
		var swerveModuleStates = kinematics.toSwerveModuleStates(
			discretizedSpeeds
		);
		SwerveDriveKinematics.desaturateWheelSpeeds(
			swerveModuleStates,
			maxSpeedMetersPerSecond
		);
		return swerveModuleStates;
	}

	/**
	 * Directly set the swerve modules to the specified states.
	 *
	 * @param desiredStates the desired swerve module states
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
			desiredStates,
			maxSpeedMetersPerSecond
		);
		for (int i = 0; i < 4; i++) {
			swerveModules[i].setDesiredState(desiredStates[i], true);
		}
	}

	/**
	 * Fixes situation where robot drifts in the direction it's rotating in if turning and translating at the same time
	 * @see https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
	 */
	private static ChassisSpeeds discretize(
		ChassisSpeeds originalChassisSpeeds
	) {
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

	/**
	 * Gets the translations of the swerve modules from the center of the robot.
	 * Used when initializing the kinematics object
	 *
	 * @return an array containing the translations of the swerve modules in the
	 *         same order they were put into the SwerveDrive constructor
	 */
	public Translation2d[] getModuleTranslations() {
		Translation2d[] translations = new Translation2d[swerveModules.length];
		for (int i = 0; i < swerveModules.length; i++) {
			translations[i] = swerveModules[i].getWheelLocationMeters();
		}
		return translations;
	}

	/**
	 * Gets the states of the swerve modules. Used primarily for kinematics
	 *
	 * @return an array containing the states of the swerve modules in the same
	 *         order they were put into the SwerveDrive constructor
	 */
	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
		for (int i = 0; i < swerveModules.length; i++) {
			states[i] = swerveModules[i].getState();
		}
		return states;
	}

	/**
	 * Gets the positions of the swerve modules. Used primarily for poseEstimator
	 *
	 * @return an array containing the positions of the swerve modules in the same
	 *         order they were put into the SwerveDrive constructor
	 */
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
		for (int i = 0; i < swerveModules.length; i++) {
			positions[i] = swerveModules[i].getPosition();
		}
		return positions;
	}

	/**
	 * Sets module states to zero
	 */
	public void zeroModules() {
		for (SwerveModule module : swerveModules) {
			module.setDesiredState(new SwerveModuleState(), true);
		}
	}
}
