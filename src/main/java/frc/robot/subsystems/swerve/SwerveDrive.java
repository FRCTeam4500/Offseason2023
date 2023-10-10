package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.Gyro;
import frc.robot.subsystems.messaging.MessagingSystem;
import java.util.function.DoubleSupplier;

/**
 * Subsystem class which represents the drivetrain of our robot
 * <p> Used to move the robot chassis
 */
public class SwerveDrive extends SubsystemBase implements SwerveDriveInterface {

	private Gyro gyro;
	private SwerveModule[] modules;
	private SwerveDriveKinematics kinematics;
	private SwerveDriveOdometry odometry;
	private static SwerveDrive instanceSwerve;
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
				0.2,
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
		odometry =
			new SwerveDriveOdometry(
				kinematics,
				gyro.getRotation2d(),
				getModulePositions(),
				new Pose2d()
			);
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
		odometry.update(gyro.getRotation2d(), getModulePositions());
		// if (vision.hasValidTargets(0)) {
		// 	poseEstimator.addVisionMeasurement(
		// 		vision.getRobotPose(0),
		// 		Timer.getFPGATimestamp()
		// 	);
		// }
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
			// targetChassisSpeeds
		);
		driveModules(states);
	}

	public void driveModules(SwerveModuleState[] states) {
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

	/*
	 * EXPERIMENTAL START
	 */

	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

	public ProfiledPIDController headingController = new ProfiledPIDController(
		1.2,
		0,
		0.1,
		new Constraints(Math.PI * 4, Math.PI * 6)
	);

	public static final Constraints constraints = new Constraints(
		SwerveConstants.MAX_LINEAR_SPEED,
		SwerveConstants.MAX_LINEAR_ACCELERATION
	);

	public static final double kPXController = 7.0;
	public static final double kPYController = 7.0;
	public static final double kPThetaController = 1.9;
	public static final double kDThetaController = 0.0;

	public static final ProfiledPIDController xController = new ProfiledPIDController(
		kPXController,
		0,
		0,
		constraints
	);
	public static final ProfiledPIDController yController = new ProfiledPIDController(
		kPYController,
		0,
		0,
		constraints
	);

	public static final SimpleMotorFeedforward thetaFeedForward = new SimpleMotorFeedforward(
		0.24,
		1.0e+3
	);
	/* Constraint for the motion profilied robot angle controller */
	public static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
		SwerveConstants.MAX_ROTATIONAL_SPEED,
		SwerveConstants.MAX_ROTATIONAL_ACCELERATION
	);

	public double deadband(double value, double minumum) {
		if (Math.abs(value) < minumum) {
			return 0;
		}
		return value;
	}

	/**
	 * Set the modules to the correct state based on a desired translation and rotation, either field
	 * or robot relative and either open or closed loop
	 */
	public void drive(
		Translation2d translation,
		double rotation,
		boolean fieldRelative,
		boolean isOpenLoop,
		boolean useAlliance
	) {
		Pose2d velPose = new Pose2d(
			translation.times(0.02),
			new Rotation2d(rotation * 0.02)
		);
		Twist2d velTwist = new Pose2d().log(velPose);
		SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(
					velTwist.dx / 0.02,
					velTwist.dy / 0.02,
					velTwist.dtheta / 0.02,
					useAlliance &&
						DriverStation.getAlliance() ==
						DriverStation.Alliance.Red
						? new Rotation2d(getRobotAngle()) // TODO: CHECK: CCW+, CW-
						: new Rotation2d(getRobotAngle())
				)
				: new ChassisSpeeds(
					velTwist.dx / 0.02,
					velTwist.dy / 0.02,
					velTwist.dtheta / 0.02
				)
		);
		SwerveDriveKinematics.desaturateWheelSpeeds(
			swerveModuleStates,
			SwerveConstants.MAX_LINEAR_SPEED
		);

		for (int i = 0; i < 4; i++) {
			modules[i].setDesiredState(swerveModuleStates[i], isOpenLoop);
		}

		chassisSpeeds =
			new ChassisSpeeds(velTwist.dx, velTwist.dy, velTwist.dtheta);
	}

	/** Generates a Command that consumes an X, Y, and Theta input supplier to drive the robot */
	public CommandBase driveCommand(
		DoubleSupplier x,
		DoubleSupplier y,
		DoubleSupplier omega,
		boolean fieldRelative,
		boolean isOpenLoop,
		boolean useAlliance
	) {
		return new RunCommand(
			() ->
				drive(
					new Translation2d(x.getAsDouble(), y.getAsDouble())
						.times(SwerveConstants.MAX_LINEAR_SPEED),
					omega.getAsDouble() * SwerveConstants.MAX_ROTATIONAL_SPEED,
					fieldRelative,
					isOpenLoop,
					useAlliance
				),
			this
		);
	}

	public Command headingLockDriveCommand(
		DoubleSupplier x,
		DoubleSupplier y,
		DoubleSupplier theta,
		boolean fieldRelative,
		boolean isOpenLoop
	) {
		return driveCommand(
			x,
			y,
			() ->
				headingController.calculate(
					getRobotAngle(),
					theta.getAsDouble()
				),
			fieldRelative,
			isOpenLoop,
			false
		);
	}

	public Command poseLockDriveCommand(
		DoubleSupplier x,
		DoubleSupplier y,
		DoubleSupplier theta,
		boolean fieldRelative,
		boolean isOpenLoop
	) {
		return new InstantCommand(() -> {
			xController.reset(getRobotPose().getX());
			yController.reset(getRobotPose().getY());
			headingController.reset(getRobotAngle() % (Math.PI * 2));
			headingController.setGoal(theta.getAsDouble());
		})
			.andThen(
				driveCommand(
						() ->
							deadband(
								xController.calculate(
									getRobotPose().getX(),
									x.getAsDouble()
								),
								0.05
							),
						() ->
							deadband(
								yController.calculate(
									getRobotPose().getY(),
									y.getAsDouble()
								),
								0.05
							),
						() ->
							deadband(
								headingController.calculate(
									getRobotPose().getRotation().getRadians() %
									(2 * Math.PI)
								),
								0.05
							),
						fieldRelative,
						isOpenLoop,
						false
					)
					.alongWith(
						new PrintCommand(getRobotPose().getX() + " x"),
						new PrintCommand(getRobotPose().getY() + " y"),
						new PrintCommand(
							headingController.getPositionError() +
							" heading error"
						)
					)
			);
	}

	/*
	 * EXPERIMENTAL END
	 */

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

	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	/**
	 * Uses forward kinematics to turn the swerve module's states into a chassis speeds for the robot
	 * @return The robot's current chassis speeds
	 */
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	/**
	 * Gets the robot's current odometric position
	 * @return the robot's current pose
	 */
	public Pose2d getRobotPose() {
		return odometry.getPoseMeters();
	}

	/**
	 * Sets the robot's odometric position to a given position
	 * @param newPose the pose the robot should be
	 */
	public void resetPose(Pose2d newPose) {
		odometry.resetPosition(
			gyro.getRotation2d(),
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

	public void lockMovement(double turnAngle) {
		modules[0].setModuleAngle(-turnAngle);
		modules[1].setModuleAngle(turnAngle);
		modules[2].setModuleAngle(turnAngle);
		modules[3].setModuleAngle(-turnAngle);
	}

	/**
	 * Update with real values
	 * <p>Order:
	 * <p>Front Left, Front Right, Back Left, Back Right
	 */
	@Override
	public void updateInputs(DriveInputs inputs) {
		inputs.frontLeftModuleDriveVelocity =
			modules[0].getModuleState().speedMetersPerSecond;
		inputs.frontLeftModuleAngleRad =
			modules[0].getModuleState().angle.getRadians();

		inputs.frontRightModuleDriveVelocity =
			modules[1].getModuleState().speedMetersPerSecond;
		inputs.frontRightModuleAngleRad =
			modules[1].getModuleState().angle.getRadians();

		inputs.backLeftModuleDriveVelocity =
			modules[2].getModuleState().speedMetersPerSecond;
		inputs.backLeftModuleAngleRad =
			modules[2].getModuleState().angle.getRadians();

		inputs.backRightModuleDriveVelocity =
			modules[3].getModuleState().speedMetersPerSecond;
		inputs.backRightModuleAngleRad =
			modules[3].getModuleState().angle.getRadians();
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
		builder.addDoubleProperty("Gyro Pitch: ", () -> gyro.getPitch(), null);
	}
}
