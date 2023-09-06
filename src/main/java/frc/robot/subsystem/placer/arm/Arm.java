package frc.robot.subsystem.placer.arm;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.component.SparkMaxComponent;
import frc.robot.component.TalonComponent;

public class Arm extends SubsystemBase implements ArmInterface {

	private SparkMaxComponent angleMotor;
	private TalonComponent extensionMotor;
	private SparkMaxPIDController anglePIDController;
	private double targetAngle;
	private double targetExtension;

	private ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

	public ArmInputsAutoLogged getInputs() {
		return inputs;
	}

	private static Arm instanceArm = null;

	/**
	 * Creates a new Arm from Constants
	 */
	private Arm() {
		angleMotor =
			new SparkMaxComponent(
				ArmConstants.TILT_MOTOR_ID,
				ArmConstants.TILT_MOTOR_TYPE
			);
		extensionMotor = new TalonComponent(ArmConstants.WINCH_MOTOR_ID, "Talon SRX");

		angleMotor.setInverted(true);

		anglePIDController = angleMotor.getPIDController();
		anglePIDController.setP(0.04);
		anglePIDController.setI(0);
		anglePIDController.setD(0);
		anglePIDController.setOutputRange(-.5, .5);

		extensionMotor.config_kP(0, .4);
		extensionMotor.config_kI(0, 0);
		extensionMotor.config_kD(0, 0);
		extensionMotor.configAllowableClosedloopError(0, 400);
		extensionMotor.configForwardSoftLimitEnable(true);
		extensionMotor.configForwardSoftLimitThreshold(10000);
		extensionMotor.configPeakOutputForward(.6);
		extensionMotor.configPeakOutputReverse(-0.3);
	}

	/**
	 * Gets instance of Arm. If the Arm is null, it will create a new one.
	 * @return Arm type "Arm"
	 */
	public static synchronized Arm getInstance() {
		if (instanceArm == null) {
			instanceArm = new Arm();
		}
		return instanceArm;
	}

	/**
	 * Sets the angle of the arm
	 * @param position The new target angle of the arm, in radians
	 */
	public void setAngle(double position) {
		targetAngle = position;
		angleMotor.setAngle(position / ArmConstants.ARM_ANGLE_RATIO);
	}

	/**
	 * Changes the angle of the arm <p>
	 * Useful for when things go wrong/debugging
	 * @param addition how much the current angle should be changed, in radians
	 */
	public void changeAngle(double addition) {
		setAngle(targetAngle + addition);
	}

	/**
	 * Gets the angle of the arm
	 * @return the current angle of the arm, in radians
	 */
	public double getAngle() {
		return angleMotor.getAngle()  * ArmConstants.ARM_ANGLE_RATIO;
	}

	public double getTargetAngle() {
		return targetAngle;
	}

	/**
	 * Sets the extension of the arm
	 * @param position the new target extension of the arm, in meters
	 */
	public void setExtension(double position) {
		targetExtension = position;
		extensionMotor.setAngle(
			position *
			ArmConstants.ARM_RADIANS_TO_LINEAR_RATIO /
			ArmConstants.ARM_EXTENSION_RATIO
		);
	}

	/**
	 * Changes the extension of the arm <p>
	 * Useful for when things go wrong/debugging
	 * @param addition how much the current extension should be changed, in meters
	 */
	public void changeExtension(double addition) {
		setExtension(targetExtension + addition);
	}

	/**
	 * Gets the extension of the arm
	 * @return the current extension of the arm, in meters
	 */
	public double getExtension() {
		return (
			extensionMotor.getAngle() *
			ArmConstants.ARM_EXTENSION_RATIO /
			ArmConstants.ARM_RADIANS_TO_LINEAR_RATIO
		);
	}

	public double getTargetExtension() {
		return targetExtension;
	}

	public void updateInputs(ArmInputs inputs) {
		inputs.extentionMotorRot = getExtension();
		inputs.tiltMotorRot = getAngle();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty(
			"Target Angle: ",
			() -> getTargetAngle(),
			null
		);
		builder.addDoubleProperty(
			"Target Extension: ",
			() -> getTargetExtension(),
			null
		);

		builder.addDoubleProperty("Current Angle: ", () -> getAngle(), null);
		builder.addDoubleProperty(
			"Current Extension: ",
			() -> getExtension(),
			null
		);
	}
}
