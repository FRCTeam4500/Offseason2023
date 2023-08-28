package frc.robot.subsystem.placer.intake;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.IntakeConstants;
import frc.robot.component.SparkMaxComponent;
import java.util.function.Supplier;

public class Intake extends SubsystemBase implements IntakeInterface {

	private SparkMaxComponent outputMotor;
	private SparkMaxComponent angleMotor;
	private SparkMaxPIDController anglePIDController;
	private static GamePiece gamePiece = GamePiece.Cone;
	private double targetAngle;
	private double targetOutput;

	private IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

	public IntakeInputsAutoLogged getInputs() {
		return inputs;
	}

	private static Intake instanceIntake = null;

	private Intake() {
		outputMotor =
			new SparkMaxComponent(
				IntakeConstants.INTAKE_MOTOR_ID,
				IntakeConstants.INTAKE_MOTOR_TYPE
			);
		angleMotor =
			new SparkMaxComponent(
				IntakeConstants.INTAKE_ANGLE_MOTOR_ID,
				IntakeConstants.ANGLE_MOTOR_TYPE
			);

		outputMotor.setIdleMode(IdleMode.kBrake);

		angleMotor.setIdleMode(IdleMode.kCoast);
		angleMotor.setSoftLimit(SoftLimitDirection.kReverse, -40);

		anglePIDController = angleMotor.getPIDController();
		anglePIDController.setP(1);
		anglePIDController.setI(0);
		anglePIDController.setD(0);
		anglePIDController.setOutputRange(-.3, .3);
	}

	/**
	 * Gets instance of Intake. If the Intake is null, it will create a new one.
	 * @return
	 */
	public static synchronized Intake getInstance() {
		if (instanceIntake == null) {
			instanceIntake = new Intake();
		}
		return instanceIntake;
	}

	public void setOutput(double output) {
		targetOutput = output;
		outputMotor.set(output);
	}

	public double getOutput() {
		return outputMotor.getOutput();
	}

	public double getTargetOutput() {
		return targetOutput;
	}

	/**
	 * Sets the angle of the intake
	 * @param angle the new target angle of the intake, in radians
	 */
	public void setAngle(double angle) {
		targetAngle = angle;
		angleMotor.setAngle(angle / IntakeConstants.INTAKE_ANGLE_RATIO);
	}

	/**
	 * Changes the angle of the intake <p>
	 * Useful for when things go wrong/debugging
	 * @param addition how much the current angle should be changed, in radians
	 */
	public void changeAngle(double addition) {
		setAngle(getAngle() + addition);
	}

	/**
	 * Gets the angle of the intake
	 * @return the current angle of the intake, in radians
	 */
	public double getAngle() {
		return angleMotor.getAngle() * IntakeConstants.INTAKE_ANGLE_RATIO;
	}

	public double getTargetAngle() {
		return targetAngle;
	}

	public static void setGamePiece(GamePiece piece) {
		Intake.gamePiece = piece;
	}

	public static Supplier<GamePiece> getGamePiece() {
		return () -> Intake.gamePiece;
	}

	@Override
	public void updateInputs(IntakeInputs inputs) {
		inputs.intakeAngleMotorRot = getAngle();
		inputs.intakeOutput = getOutput();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty(
			"Target Angle: ",
			() -> getTargetAngle(),
			null
		);
		builder.addDoubleProperty(
			"Target Percent Output: ",
			() -> getTargetOutput(),
			null
		);
		builder.addDoubleProperty(
			"Curent Angle: ", 
			() -> getAngle(), 
			null
		);
		builder.addDoubleProperty(
			"Current Percent Output: ",
			() -> getOutput(),
			null
		);
		builder.addStringProperty(
			"Current Game Piece: ",
			() -> gamePiece.name(),
			null
		);
	}
}
