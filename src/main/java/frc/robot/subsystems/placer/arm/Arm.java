package frc.robot.subsystems.placer.arm;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EnumConstants.TalonType;
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

	public ArmInputsAutoLogged getInputs() {
		return inputs;
	}

	private Arm() {
		angleMotor =
			new SparkMaxMotorController(
				ArmConstants.ANGLE_MOTOR_ID,
				MotorType.kBrushless
			);
		extensionMotor =
			new TalonMotorController(
				ArmConstants.EXTENSION_MOTOR_ID,
				TalonType.TalonSRX
			);

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

	public static synchronized Arm getInstance() {
		if (instanceArm == null) {
			instanceArm = new Arm();
		}
		return instanceArm;
	}

	public void setAngle(double angle) {
		targetAngle = angle;
		angleMotor.setAngle(angle);
	}

	public void changeAngle(double addition) {
		setAngle(targetAngle + addition);
	}

	public double getAngle() {
		return angleMotor.getAngle();
	}

	private double getTargetAngle() {
		return targetAngle;
	}

	public void setExtension(double position) {
		targetExtension = position;
		extensionMotor.setAngle(position);
	}

	public void changeExtension(double addition) {
		setExtension(targetExtension + addition);
	}

	public double getExtension() {
		return extensionMotor.getAngle();
	}

	private double getTargetExtension() {
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
