package frc.robot.subsystems.swervydwervy;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.hardware.TalonMotorController;
import frc.robot.utilities.HelperMethods;

public class SwerveModule {
    final int k100msPerSecond = 10;

    private final TalonMotorController angleMotor;
    private final TalonMotorController driveMotor;
    private final double angleCountsPerRev;
    private final double driveCountsPerRev;
    private final double driveGearRatio;
    private final double wheelCircumferenceMeters;
    private final double driveDeadbandMetersPerSecond;
    private final double driveMaximumMetersPerSecond;
    private final Translation2d wheelLocationMeters; // forward pos x, left pos y

    private Rotation2d previousAngle = new Rotation2d();

    public SwerveModule(TalonMotorController driveMotor, TalonMotorController angleMotor,
            Translation2d wheelLocationMeters) {
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
        angleCountsPerRev = 2048; // Should be for FX
        driveCountsPerRev = 2048;
        driveGearRatio = 5.;
        wheelCircumferenceMeters = Math.PI * Units.inchesToMeters(3);
        driveDeadbandMetersPerSecond = 0.05;
        driveMaximumMetersPerSecond = 5.;
        this.wheelLocationMeters = wheelLocationMeters;
        resetDriveEncoder();
    }

    public double getMaxSpeedMetersPerSecond() {
        return driveMaximumMetersPerSecond;
    }

    public Translation2d getWheelLocationMeters() {
        return wheelLocationMeters;
    }

    public double getDriveCountsPerRev() {
        return driveCountsPerRev;
    }

    public SwerveModuleState getState() {
        double speedMetersPerSecond = getDriveMetersPerSecond();
        Rotation2d angle = getAngleMotorRotation2d();
        return new SwerveModuleState(speedMetersPerSecond, angle);
    }

    public SwerveModulePosition getPosition() {
        double wheelPositionMeters = getDrivePositionMeters();
        Rotation2d angle = getAngleMotorRotation2d();
        return new SwerveModulePosition(wheelPositionMeters, angle);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isDriveOpenLoop) {
        assert desiredState.speedMetersPerSecond >= 0.0;

        if (desiredState.speedMetersPerSecond < driveDeadbandMetersPerSecond) {
            desiredState = new SwerveModuleState(0.0, previousAngle);
        }

        SwerveModuleState optimizedState = setAngleOptimizedState(desiredState);

        if (isDriveOpenLoop) {
            setDriveOpenLoopMetersPerSecond(optimizedState.speedMetersPerSecond);
        } else {
            setDriveClosedLoopMetersPerSecond(optimizedState.speedMetersPerSecond);
        }
    }

    public void resetDriveEncoder() {
        driveMotor.setSelectedSensorPosition(0);
    }

    public TalonMotorController getAngleMotor() {
        return angleMotor;
    }

    public BaseTalon getDriveMotor() {
        return driveMotor;
    }

    public Rotation2d getAngleMotorRotation2d() {
        double azimuthCounts = angleMotor.getSelectedSensorPosition();
        double radians = 2.0 * Math.PI * azimuthCounts / angleCountsPerRev;
        return new Rotation2d(radians);
    }

    public void setAngleRotation2D(Rotation2d angle) {
        setAngleOptimizedState(new SwerveModuleState(0.0, angle));
    }

    private SwerveModuleState setAngleOptimizedState(SwerveModuleState desiredState) {
        // minimize change in heading by potentially reversing the drive direction
        Rotation2d currentAngle = getAngleMotorRotation2d();
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentAngle);

        // set the azimuth wheel position
        double countsBefore = angleMotor.getSelectedSensorPosition();
        double countsFromAngle = optimizedState.angle.getRadians() / (2.0 * Math.PI) * angleCountsPerRev;
        double countsDelta = Math.IEEEremainder(countsFromAngle - countsBefore, angleCountsPerRev);
        angleMotor.set(ControlMode.MotionMagic, countsBefore + countsDelta);

        // save previous angle for use if inside deadband in setDesiredState()
        previousAngle = optimizedState.angle;
        return optimizedState;
    }

    private double getDriveMetersPerSecond() {
        double encoderCountsPer100ms = driveMotor.getSelectedSensorVelocity();
        double motorRotationsPer100ms = encoderCountsPer100ms / driveCountsPerRev;
        double wheelRotationsPer100ms = motorRotationsPer100ms * driveGearRatio;
        double metersPer100ms = wheelRotationsPer100ms * wheelCircumferenceMeters;
        return metersPer100ms * k100msPerSecond;
    }

    private double getDrivePositionMeters() {
        double encoderTicks = driveMotor.getSelectedSensorPosition();
        double motorPositionTicks = encoderTicks / driveCountsPerRev;
        double wheelPositionTicks = motorPositionTicks * driveGearRatio;
        double wheelPositionMeters = wheelPositionTicks * wheelCircumferenceMeters;
        return wheelPositionMeters;
    }

    private void setDriveOpenLoopMetersPerSecond(double metersPerSecond) {
        driveMotor.set(ControlMode.PercentOutput,
                HelperMethods.clamp(-0.7, 0.7, metersPerSecond / driveMaximumMetersPerSecond)); // TODO: MINMAX
    }

    private void setDriveClosedLoopMetersPerSecond(double metersPerSecond) { // Gets the robot going at specified
                                                                             // velocity
        double wheelRotationsPerSecond = metersPerSecond / wheelCircumferenceMeters;
        double motorRotationsPerSecond = wheelRotationsPerSecond / driveGearRatio;
        double encoderCountsPerSecond = motorRotationsPerSecond * driveCountsPerRev;
        driveMotor.set(ControlMode.Velocity, encoderCountsPerSecond / k100msPerSecond);
    }

    private int getWheelIndex() {
        if (wheelLocationMeters.getX() > 0 && wheelLocationMeters.getY() > 0) { // Front Left
            return 0;
        }
        if (wheelLocationMeters.getX() > 0 && wheelLocationMeters.getY() < 0) { // Front Right
            return 1;
        }
        if (wheelLocationMeters.getX() < 0 && wheelLocationMeters.getY() > 0) { // Back Left
            return 2;
        }
        return 3; // Back Right
    }

    @Override
    public String toString() {
        return "SwerveModule{" + getWheelIndex() + '}';
    }

}
