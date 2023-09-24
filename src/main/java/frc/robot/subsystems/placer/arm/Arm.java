package frc.robot.subsystems.placer.arm;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.hardware.SparkMaxMotorController;
import frc.robot.hardware.TalonMotorController;

public class Arm extends SubsystemBase implements ArmInterface {
	private SparkMaxMotorController angleMotor;
	private TalonMotorController extensionMotor;
	private SparkMaxPIDController anglePIDController;
	private double targetAngle;
	private double targetExtension;
	private static Arm instanceArm = null;
	private ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

	/** Gets inputs for logging */
	public ArmInputsAutoLogged getInputs() {
		return inputs;
	}

	/** Creates a new arm */
	private Arm() {
		angleMotor =
			new SparkMaxMotorController(
				ArmConstants.TILT_MOTOR_ID,
				ArmConstants.TILT_MOTOR_TYPE
			);
		extensionMotor = new TalonMotorController(ArmConstants.WINCH_MOTOR_ID, "Talon SRX");

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

	/** Gets global arm instance*/
	public static synchronized Arm getInstance() {
		if (instanceArm == null) {
			instanceArm = new Arm();
		}
		return instanceArm;
	}

	/** Sets arm angle */
	public void setAngle(double angle) {
		targetAngle = angle;
		angleMotor.setAngle(angle);
	}

	/** Changes arm angle */
	public void changeAngle(double addition) {
		setAngle(targetAngle + addition);
	}

	/** Gets arm angle */
	public double getAngle() {
		return angleMotor.getAngle();
	}

	/** Gets target arm angle */
	private double getTargetAngle() {
		return targetAngle;
	}

	/** Sets the arm extension */
	public void setExtension(double position) {
		targetExtension = position;
		extensionMotor.setAngle(position);
	}

	/** Changes arm extension*/
	public void changeExtension(double addition) {
		setExtension(targetExtension + addition);
	}

	/** Gets arm extension*/
	public double getExtension() {
		return extensionMotor.getAngle();
	}

	/** Gets target arm extension */
	private double getTargetExtension() {
		return targetExtension;
	}

	/** Updates inputs for logging */
	public void updateInputs(ArmInputs inputs) {
		inputs.extentionMotorRot = getExtension();
		inputs.tiltMotorRot = getAngle();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Target Angle: ", () -> getTargetAngle(), null);
		builder.addDoubleProperty("Target Extension: ", () -> getTargetExtension(), null );
		builder.addDoubleProperty("Current Angle: ", () -> getAngle(), null);
		builder.addDoubleProperty("Current Extension: ", () -> getExtension(), null);
	}
}
