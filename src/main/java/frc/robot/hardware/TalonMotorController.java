package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import frc.robot.hardware.interfaces.SwerveMotorController;
import frc.robot.subsystems.messaging.MessagingSystem;

public class TalonMotorController extends BaseTalon implements SwerveMotorController{
    private double TICKS_PER_RADIAN;

    public TalonMotorController(int deviceID, String motorModel) {
        super(deviceID, motorModel);
        switch (motorModel) {
            case "Talon FX":
                TICKS_PER_RADIAN = 2048 / Math.PI / 2;
                break;
            case "Talon SRX": 
                TICKS_PER_RADIAN = 4096 / Math.PI / 2;
                break;
            default:
                MessagingSystem.getInstance().addMessage("Can ID #" + deviceID + "'s motor model isn't a valid option");
        }
    }

    public void setOutput(double targetPercentOutput) {
        set(ControlMode.PercentOutput, targetPercentOutput);
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
        if (TICKS_PER_RADIAN == 4096 / Math.PI / 2) {
            set(ControlMode.Position, targetAngle * TICKS_PER_RADIAN);
        } else {
            set(ControlMode.MotionMagic, targetAngle * TICKS_PER_RADIAN);
        }
    }

    public double getAngle() {
        return getSelectedSensorPosition() / TICKS_PER_RADIAN;
    }

    public void configureForSwerve(boolean isInverted, int currentLimit, double kP, double kD, boolean isDriveMotor){
        if (isDriveMotor) {
            configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit + 1, 0.1),
			50);
            config_kP(0, kP);
            config_kF(0, 0.047);
            config_IntegralZone(0, 0);
            setInverted(isInverted);
        } else {
            setInverted(isInverted);
            config_kP(0, kP);
            configMotionCruiseVelocity(10000);
            configMotionAcceleration(10000);
            configAllowableClosedloopError(0, 0);
            configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit + 1, 0.1), 50);
            configClearPositionOnQuadIdx(true, 10);
        }
    }
}
