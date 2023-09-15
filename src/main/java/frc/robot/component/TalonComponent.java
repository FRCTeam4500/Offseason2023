package frc.robot.component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public class TalonComponent extends BaseTalon implements SwerveMotor{
    private double TICKS_PER_RADIAN;

    public TalonComponent(int deviceID, String motorModel) {
        super(deviceID, motorModel);
        switch (motorModel) {
            case "Talon FX":
                TICKS_PER_RADIAN = 2048 / Math.PI / 2;
                break;
            case "Talon SRX": 
                TICKS_PER_RADIAN = 4096 / Math.PI / 2;
                break;
            default:
        }
    }

    public void setOutput(double targetPercentOutput) {
        set(ControlMode.PercentOutput, m_handle);
    }

    public double getOutput() {
        return getMotorOutputPercent();
    }

    public void setAngularVelocity(double targetAngularVelocity) {
        set(ControlMode.Velocity, targetAngularVelocity * TICKS_PER_RADIAN / 10.0);
    }

    public double getAngularVelocity() {
        return getSelectedSensorVelocity() / TICKS_PER_RADIAN * 10;
    }

    public void setAngle(double targetAngle) {
        set(ControlMode.Position, targetAngle * TICKS_PER_RADIAN);
    }

    public double getAngle() {
        return getSelectedSensorPosition() / TICKS_PER_RADIAN;
    }

    public void configureForSwerve(boolean isInverted, int currentLimit, double kP, double kD, boolean isDriveMotor){
        configSupplyCurrentLimit(
			new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit + 1, 0.1), 
            50);
        config_kP(0, kP);
        config_kI(0, 0);
        config_kD(0, kD);
        config_kF(0, 0);
        if (isDriveMotor) {
            config_IntegralZone(0, 0);
        } else {
            configAllowableClosedloopError(0, 0);
            configClearPositionOnQuadIdx(true, 10);
        }
    }
}
