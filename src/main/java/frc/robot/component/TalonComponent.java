package frc.robot.component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public class TalonComponent extends BaseTalon implements GenericMotor{
    private double TICKS_PER_RADIAN;

    public TalonComponent(int deviceID, String motorModel) {
        super(deviceID, motorModel);
        if (motorModel == "Talon SRX") {
            TICKS_PER_RADIAN = 4096 / Math.PI / 2;
        } else {
            TICKS_PER_RADIAN = 2048 / Math.PI / 2;
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
}
