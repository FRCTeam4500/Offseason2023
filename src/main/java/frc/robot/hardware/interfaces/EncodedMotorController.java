package frc.robot.hardware.interfaces;

public interface EncodedMotorController {
	/** Units are radians/sec */
	public void setAngularVelocity(double targetAngularVelocity);

	/** Units are radians/sec */
	public double getAngularVelocity();

	/** Units are radians */
	public void setAngle(double targetAngle);

	/** Units are radians */
	public double getAngle();

	/** Units are percent */
	public void setOutput(double targetOutput);

	/** Units are percent */
	public double getOutput();
}
