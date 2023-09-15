package frc.robot.component;

public interface SwerveMotor extends GenericMotor{
    public void configureForSwerve(boolean isInverted, int currentLimit, double kP, double kD, boolean isDriveMotor);
}
