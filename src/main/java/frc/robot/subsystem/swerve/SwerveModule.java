package frc.robot.subsystem.swerve;


import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.SwerveConstants;
import frc.robot.component.TalonFXComponent;

public class SwerveModule {
    private TalonFXComponent driveMotor;
    private TalonFXComponent angleMotor;
    private Translation2d translationFromCenter;

    public SwerveModule(int driveId, int angleId, Translation2d translationToCenter, 
    boolean invertDrive, boolean invertAngle, 
    double drivekP, double anglekP) {
        angleMotor = new TalonFXComponent(angleId);
        angleMotor.setInverted(invertAngle);
        angleMotor.config_kP(0, anglekP);
        angleMotor.configMotionCruiseVelocity(10000);
        angleMotor.configMotionAcceleration(10000);
        angleMotor.configAllowableClosedloopError(0, 0);
        angleMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 26, 0.1));
        angleMotor.configClearPositionOnQuadIdx(true, 10);

        driveMotor = new TalonFXComponent(driveId);
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 36, 0.1));
        driveMotor.config_kP(0, drivekP);
        driveMotor.config_kF(0, 0.047);
        driveMotor.config_IntegralZone(0, 0);
        driveMotor.setInverted(invertDrive);

        this.translationFromCenter = translationToCenter;
    }

    public void drive(SwerveModuleState initialTargetState) {
        SwerveModuleState targetState = SwerveModuleState.optimize(initialTargetState, getModuleState().angle);
        driveMotor.setAngularVelocity(targetState.speedMetersPerSecond * 2 / (SwerveConstants.DRIVE_RATIO * SwerveConstants.WHEEL_DIAMETER)); // Math seems wierd because it is calculating angular velocity in radians/second
        angleMotor.setAngle(targetState.angle.getRadians() / SwerveConstants.ANGLE_RATIO);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(driveMotor.getAngularVelocity() * SwerveConstants.DRIVE_RATIO * SwerveConstants.WHEEL_DIAMETER / 2, new Rotation2d(angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveMotor.getMotorRotations() * SwerveConstants.DRIVE_RATIO * SwerveConstants.WHEEL_DIAMETER * Math.PI, getModuleState().angle);
    }

    public Translation2d getTranslationFromCenter() {
        return translationFromCenter;
    }

    public void setModuleAngle(double targetAngle) {
        angleMotor.setAngle(targetAngle / SwerveConstants.ANGLE_RATIO);
    }

    public void setModuleOutput(double targetOutput) {
        driveMotor.setOutput(targetOutput);
    }
}
