package frc.robot.subsystem.swerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.component.hardware.TalonFXComponent;

public class SwerveModule {
    private TalonFXComponent driveMotor;
    private TalonFXComponent angleMotor;
    private Translation2d translationFromCenter;
    public SwerveModule(int driveId, int angleId, Translation2d translationToCenter, 
    boolean invertDrive, boolean invertAngle, 
    double drivekP, double anglekP) {
        TalonFXComponent angleMotor = new TalonFXComponent(angleId);
        angleMotor.setInverted(invertAngle);
        angleMotor.config_kP(0, anglekP);
        angleMotor.configMotionCruiseVelocity(10000);
        angleMotor.configMotionAcceleration(10000);
        angleMotor.configAllowableClosedloopError(0, 0);
        angleMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 26, 0.1));
        angleMotor.configClearPositionOnQuadIdx(true, 10);

        TalonFXComponent driveMotor = new TalonFXComponent(driveId);
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 36, 0.1));
        driveMotor.config_kP(0, drivekP);
        driveMotor.config_kF(0, 0.047);
        driveMotor.config_IntegralZone(0, 0);
        driveMotor.setInverted(invertDrive);

        this.translationFromCenter = translationToCenter;
    }

    public void drive(SwerveModuleState initialTargetState) {
        SwerveModuleState targetState = initialTargetState.optimize(initialTargetState, getModuleState().angle);
        
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(driveMotor.getAngularVelocity() * SwerveConstants.DRIVE_RATIO * SwerveConstants.WHEEL_DIAMETER / 2, new Rotation2d(angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO));
    }
}
