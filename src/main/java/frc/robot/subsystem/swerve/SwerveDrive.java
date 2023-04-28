package frc.robot.subsystem.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.component.hardware.AHRSAngleGetterComponent;

public class SwerveDrive extends SubsystemBase{
    private AHRSAngleGetterComponent gyro = new AHRSAngleGetterComponent(I2C.Port.kMXP);
    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;
    private double currentGyroZero;
    public SwerveDrive() {
        SwerveModule[] modules = {
            new SwerveModule(SwerveConstants.DFLPORT, SwerveConstants.AFLPORT, new Translation2d(SwerveConstants.DRIVE_Y_FRONT_TRANSLATION, SwerveConstants.DRIVE_X_LEFT_TRANSLATION), true, false, 0.1, 0.3),
            new SwerveModule(SwerveConstants.DFRPORT, SwerveConstants.AFRPORT, new Translation2d(SwerveConstants.DRIVE_Y_FRONT_TRANSLATION, SwerveConstants.DRIVE_X_RIGHT_TRANSLATION), false, false, 0.1, 0.3),
            new SwerveModule(SwerveConstants.DBLPORT, SwerveConstants.ABLPORT, new Translation2d(SwerveConstants.DRIVE_Y_BACK_TRANSLATION, SwerveConstants.DRIVE_X_LEFT_TRANSLATION), true, false, 0.1, 0.3),
            new SwerveModule(SwerveConstants.DBRPORT, SwerveConstants.ABRPORT, new Translation2d(SwerveConstants.DRIVE_Y_BACK_TRANSLATION, SwerveConstants.DRIVE_X_RIGHT_TRANSLATION), false, false, 0.1, 0.3),
        };
        this.modules = modules;
        currentGyroZero = 0;
        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getAngle()), getModulePositions());
    }

    @Override
    public void periodic() {
        odometry.update(new Rotation2d(gyro.getAngle()), getModulePositions());
    }

    public void driveFieldCentric(double forwardVelocity, double sidewaysVelocity, double rotationalVelocity) {
        driveModules(ChassisSpeeds.fromFieldRelativeSpeeds(forwardVelocity, sidewaysVelocity, rotationalVelocity, new Rotation2d(getRobotAngle())));
    }

    public void driveRobotCentric(double forwardVelocity, double sidewaysVelocity, double rotationalVelocity) {
        driveModules(new ChassisSpeeds(forwardVelocity, sidewaysVelocity, rotationalVelocity));
    }

    public void driveModules(ChassisSpeeds targetChassisSpeeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_LINEAR_SPEED);
        for (int i = 0; i < states.length; i++) {
            modules[i].drive(states[i]);
        }
    }

    public Translation2d[] getModuleTranslations() {
        Translation2d[] translations = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            translations[i] = modules[i].getTranslationFromCenter();
        }
        return translations;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getModuleState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getRobotPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(new Rotation2d(gyro.getAngle()), getModulePositions(), pose);
    }

    public double getRobotAngle() {
        return gyro.getAngle() - currentGyroZero;
    }

    public void resetRobotAngle(double offset) {
        currentGyroZero = gyro.getAngle() - offset;
    }

    public void resetRobotAngle() {
        resetRobotAngle(0);
    }

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
