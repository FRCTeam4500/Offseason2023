package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.subsystems.placer.arm.Arm;
import frc.robot.subsystems.placer.intake.Intake;
import frc.robot.Constants.IntakeConstants;

/**
 * A command that sets the Arm winch state, the Arm angle, and the Intake angle
 */
public class SetArmAndIntakeCommand extends CommandBase {

	private Arm arm;
	private Intake intake;
	private ArmPosition position;

	private double targetWinchPosition;
	private double targetArmAngle;
	private double targetIntakeAngle;

	/**
	 * The constructor of SetArmAndIntakeCommand
	 * @param arm the arm subsystem
	 * @param intake the intake subsystem
	 * @param state the desired state
	 */
	public SetArmAndIntakeCommand(ArmPosition position) {
		this.position = position;
	}

	public void initialize() {
		arm = Arm.getInstance();
		intake = Intake.getInstance();

		switch(position) {
			case Start:
				targetWinchPosition = ArmConstants.ARM_EXTENSION_ZERO;
				targetArmAngle = 0;
				targetIntakeAngle = IntakeConstants.INTAKE_ZERO_TILT;
				break;
			case Zero:
				targetWinchPosition = ArmConstants.ARM_EXTENSION_ZERO;
				targetArmAngle = ArmConstants.ARM_TILT_PLACE;
				targetIntakeAngle = IntakeConstants.INTAKE_ZERO_TILT;
				break;
			case Bot:
				// targetWinchPosition = ArmConstants.ARM_PICKUP;
				// targetArmAngle = ArmConstants.ARM_GROUND_ANGLE;
				// targetIntakeAngle = IntakeConstants.INTAKE_BOT_ANGLE;
				// break;
			case Mid:
				targetWinchPosition = ArmConstants.ARM_EXTENTION_MIDDLE;
				targetArmAngle = ArmConstants.ARM_TILT_PLACE;
				targetIntakeAngle = IntakeConstants.INTAKE_PLACE_ANGLE;
				break;
			case Top:
				targetWinchPosition = ArmConstants.ARM_EXTENTION_HIGH;
				targetArmAngle = ArmConstants.ARM_TILT_PLACE;
				targetIntakeAngle = IntakeConstants.INTAKE_HIGH_ANGLE;
				break;
			case Sub:
				targetWinchPosition = ArmConstants.ARM_EXTENTION_SUBSTATION;
				targetArmAngle = ArmConstants.ARM_TILT_SUBSTATION;
				targetIntakeAngle = IntakeConstants.INTAKE_PLACE_ANGLE;
				break;
		}
		arm.setExtension(targetWinchPosition);
		arm.setAngle(targetArmAngle);
		intake.setAngle(targetIntakeAngle);
	}

	public boolean isFinished() {
		return true;
	}
}
