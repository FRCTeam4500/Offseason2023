package frc.robot.subsystem.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.component.AHRSAngleGetterComponent;
/**
 * Subsystem class which represents the drivetrain of our robot
 * <p> Used to move the robot chassis
 */
public class SwerveDrive extends SubsystemBase{
    /**
     * The gyroscope of the robot. Used to get the absolute angle of the robot relative to its angle on startup
     */
    private AHRSAngleGetterComponent gyro;
    /**
     * An array containing the swerve drive modules of the robot. 
     * The order of the modules in the array is dependent on the order they are made in the SwerveDrive constructor
     */
    private SwerveModule[] modules;
    /** 
     * An object containing the kinematics of the swerve modules (where they are relative to the center of the robot). 
     * <p>Used to calulate module states based on a target chassis speeds and vice versa
     */
    private SwerveDriveKinematics kinematics;
    /**
     * An object which calculates the robot's position based on the positions of the swerve modules.
     * <p> Used for autonomous driving to correct for errors
     */
    private SwerveDriveOdometry odometry;
    /**
     * The angle of the current zero relative to the angle of the gyroscope. This is in radians
     */
    private double currentGyroZero;
    private PIDController forwardVelocityController;
    private PIDController sidewaysVelocityController;
    private ProfiledPIDController rotationalVelocityController;
    /**
     * Creates a new Swerve Drive with 4 swerve modules, which uses 8 falcon motors. kP of the motors and whether to invert them is set here. Everything else is pulled from constants
     */
    public SwerveDrive() {
        SwerveModule[] modules = {
            new SwerveModule(SwerveConstants.DFLPORT, SwerveConstants.AFLPORT, new Translation2d(SwerveConstants.DRIVE_Y_FRONT_TRANSLATION, SwerveConstants.DRIVE_X_LEFT_TRANSLATION), true, false, 0.1, 0.3),
            new SwerveModule(SwerveConstants.DFRPORT, SwerveConstants.AFRPORT, new Translation2d(SwerveConstants.DRIVE_Y_FRONT_TRANSLATION, SwerveConstants.DRIVE_X_RIGHT_TRANSLATION), false, false, 0.1, 0.3),
            new SwerveModule(SwerveConstants.DBLPORT, SwerveConstants.ABLPORT, new Translation2d(SwerveConstants.DRIVE_Y_BACK_TRANSLATION, SwerveConstants.DRIVE_X_LEFT_TRANSLATION), true, false, 0.1, 0.3),
            new SwerveModule(SwerveConstants.DBRPORT, SwerveConstants.ABRPORT, new Translation2d(SwerveConstants.DRIVE_Y_BACK_TRANSLATION, SwerveConstants.DRIVE_X_RIGHT_TRANSLATION), false, false, 0.1, 0.3),
        };
        this.modules = modules;
        currentGyroZero = 0;
        forwardVelocityController = new PIDController(1, 0, 0);
        sidewaysVelocityController = new PIDController(1,0, 0);
        rotationalVelocityController = new ProfiledPIDController(1, 0, 0, new Constraints(SwerveConstants.MAX_LINEAR_SPEED, SwerveConstants.MAX_LINEAR_ACCELERATION));
        gyro = new AHRSAngleGetterComponent(I2C.Port.kMXP);
        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getAngle()), getModulePositions());
    }

    /**
     * A method from Subsystem Base. <p>Our code probably shouldn't call this. This is called every scheduler tick (20ms) by WPI. Code that needs to be run repeadly should be put here
     */
    @Override
    public void periodic() {
        odometry.update(new Rotation2d(gyro.getAngle()), getModulePositions());
    }

    public ChassisSpeeds goToTargetPosition(Pose2d targetPose) {
        Pose2d currentPose = getRobotPose();
        double forwardVelocity = forwardVelocityController.calculate(currentPose.getX(), targetPose.getX());
        double sidewaysVelocity = sidewaysVelocityController.calculate(currentPose.getX(), targetPose.getY());
        double rotationalVelocity = rotationalVelocityController.calculate(getRobotAngle(), targetPose.getRotation().getRadians());
        return new ChassisSpeeds(forwardVelocity, sidewaysVelocity, rotationalVelocity);
    }

    /**
     * Drives the robot relative to the field (forward is away from the alliance wall) based on target forward, sideways, and rotational velocities
     * @param forwardVelocity the target forward velocity of the robot. Units are m/s and forward is positive
     * @param sidewaysVelocity the target sideways velocity of the robot. Units are m/s and left is positive
     * @param rotationalVelocity the target rotational velocity of the robot. Units are rad/s and counter-clockwise is positive
     */
    public void driveFieldCentric(double forwardVelocity, double sidewaysVelocity, double rotationalVelocity) {
        driveModules(ChassisSpeeds.fromFieldRelativeSpeeds(forwardVelocity, sidewaysVelocity, rotationalVelocity, new Rotation2d(getRobotAngle())));
    }

    /**
     * Drives the robot relative to itself based on target forward, sideways, and rotational velocities
     * @param forwardVelocity the target forward velocity of the robot. Units are m/s and forward is positive
     * @param sidewaysVelocity the target sideways velocity of the robot. Units are m/s and left is positive
     * @param rotationalVelocity the target rotational velocity of the robot. Units are rad/s and counter-clockwise is positive
     */
    public void driveRobotCentric(double forwardVelocity, double sidewaysVelocity, double rotationalVelocity) {
        driveModules(new ChassisSpeeds(forwardVelocity, sidewaysVelocity, rotationalVelocity));
    }

    /**
     * Uses inverse kinematics to convert a robot-relative set of chassis speeds into states to send to the swerve modules
     * @param targetChassisSpeeds The target robot-relative chassis speeds
     */
    public void driveModules(ChassisSpeeds targetChassisSpeeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_LINEAR_SPEED);
        for (int i = 0; i < modules.length; i++) {
            modules[i].drive(states[i]);
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
     * Gets the positions of the swerve modules. Used primarily for odometry
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
        odometry.resetPosition(new Rotation2d(gyro.getAngle()), getModulePositions(), newPose);
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
    }

    /**
     * Sets where the robot is currently facing to the field-centric zero
     */
    public void resetRobotAngle() {
        resetRobotAngle(0);
    }

    /**
     * A method from SubsystemBase. <p>Adds properties of the subsystem to shuffleboard. Our code should never call this class, that is done by WPI
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Gyro Angle: ", () -> gyro.getAngle(), null);
        builder.addDoubleProperty("Gyro Offset From Zero: ", () -> getRobotAngle() % (2 * Math.PI), null);
        builder.addDoubleProperty("Current Forward Speed: ", () -> getChassisSpeeds().vxMetersPerSecond, null);
        builder.addDoubleProperty("Current Sideways Speed: ", () -> getChassisSpeeds().vyMetersPerSecond, null);
        builder.addDoubleProperty("Current Rotational Speed: ", () -> getChassisSpeeds().omegaRadiansPerSecond, null);
        builder.addDoubleProperty("Odometric X: ", () -> getRobotPose().getX(), null);
        builder.addDoubleProperty("Odometric Y: ", () -> getRobotPose().getY(), null);
        builder.addDoubleProperty("Odometric Rotation: ", () -> getRobotPose().getRotation().getRadians(), null);
    }

}
