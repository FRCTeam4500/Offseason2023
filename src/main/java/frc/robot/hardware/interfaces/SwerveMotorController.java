package frc.robot.hardware.interfaces;

public interface SwerveMotorController extends EncodedMotorController{
    public void configureForSwerve(boolean isInverted, int currentLimit, double kP, double kD, boolean isDriveMotor);
}
