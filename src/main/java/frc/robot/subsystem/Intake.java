package frc.robot.subsystem;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.IntakeConstants;
import frc.robot.component.SparkMaxComponent;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Intake extends SubsystemBase {

	private SparkMaxComponent speedMotor;
	private SparkMaxComponent angleMotor;
	private SparkMaxPIDController anglePIDController;
	private static GamePiece gamePiece = GamePiece.UprightCone;
	private double targetAngle;
	private double targetSpeed;

	private static Intake instanceIntake = null;

	private Intake() {
		speedMotor =
			new SparkMaxComponent(
				IntakeConstants.INTAKE_MOTOR_ID,
				IntakeConstants.INTAKE_MOTOR_TYPE
			);
		angleMotor =
			new SparkMaxComponent(
				IntakeConstants.INTAKE_ANGLE_MOTOR_ID,
				IntakeConstants.ANGLE_MOTOR_TYPE
			);

		speedMotor.setIdleMode(IdleMode.kBrake);

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

	public void setSpeed(double speed) {
		targetSpeed = speed;
		speedMotor.set(speed);
	}

	public DoubleSupplier getSpeed() {
		return () -> speedMotor.getOutput();
	}

	public DoubleSupplier getTargetSpeed() {
		return () -> targetSpeed;
	}

	public void setAngle(double angle) {
		targetAngle = angle;
		angleMotor.setAngle(angle);
	}

	public void changeAngle(double addition) {
		setAngle(getAngle().getAsDouble() + addition);
	}

	public DoubleSupplier getAngle() {
		return () -> angleMotor.getAngle();
	}

	public DoubleSupplier getTargetAngle() {
		return () -> targetAngle;
	}

	public static void setGamePiece(GamePiece piece) {
		Intake.gamePiece = piece;
	}

	public static Supplier<GamePiece> getGamePiece() {
		return () -> Intake.gamePiece;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty(
			"Target Intake Tilt position",
			getTargetAngle(),
			null
		);
		builder.addDoubleProperty(
			"Target Intake Wheel output",
			getTargetSpeed(),
			null
		);
		builder.addDoubleProperty("Intake Angle", getAngle(), null);
		builder.addStringProperty(
			"Current Game Piece",
			() -> gamePiece.name(),
			null
		);
	}
}
