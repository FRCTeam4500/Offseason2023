package frc.robot.hardware.interfaces;

public interface EncodedMotorController {
	public void setAngularVelocity(double targetAngularVelocity);

	public double getAngularVelocity();

	public void setAngle(double targetAngle);

	public double getAngle();

	public void setOutput(double targetOutput);

	public double getOutput();
}
