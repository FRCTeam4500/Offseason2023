package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.placer.arm.Arm;
import frc.robot.subsystems.placer.intake.Intake;

/**
 * A command that sets the Arm winch state, the Arm angle, and the Intake angle
 */
public class SetArmAndIntakeCommand extends CommandBase {

	private Arm arm;
	private Intake intake;
	private ArmPosition position;

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

		arm.setExtension(position.getArmExtension());
		arm.setAngle(position.getArmAngle());
		intake.setAngle(position.getIntakeAngle());
	}

	public boolean isFinished() {
		return true;
	}
}
