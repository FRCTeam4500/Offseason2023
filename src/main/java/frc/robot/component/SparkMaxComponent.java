
package frc.robot.component;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;

/**
 * Wrapper for CANSparkMax motor controller which implements SmartMotorComponent
 */
public class SparkMaxComponent extends CANSparkMax {

    public SparkMaxComponent(int deviceID, MotorType type) {
        super(deviceID, type);
    }

    
    public double getAngle() {
        return getEncoder().getPosition();
    }

    
    public void setAngle(double position) {
        getPIDController().setReference(position, ControlType.kPosition);

    }

    public void setOutput(double output) {
        set(output);
    }

    public double getOutput() {
        return get();
    }

    public double getAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(getEncoder().getVelocity());
    }

    /**
     * set velocity, in rad/s
     * @param velocity angular velocity, in rad/s
     */
    public void setAngularVelocity(double velocity) {
        getPIDController().setReference(Units.radiansPerSecondToRotationsPerMinute(velocity), ControlType.kVelocity);
    }

}
