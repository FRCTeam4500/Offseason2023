package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.subsystems.placer.arm.Arm;
import frc.robot.subsystems.placer.intake.Intake;

public class SetArmAndIntakeCommand extends SequentialCommandGroup {
	private Arm arm;
	private Intake intake;
	public SetArmAndIntakeCommand(ArmPosition position) {
		intake = Intake.getInstance();
		arm = Arm.getInstance();
		addCommands(
			new InstantCommand(() -> intake.setAngle(IntakeConstants.ZERO_ANGLE)),
			new InstantCommand(() -> arm.setExtension(ArmConstants.ZERO_EXTENSION)),
			new WaitUntilCommand(() -> Math.abs(ArmConstants.ZERO_EXTENSION - arm.getExtension()) < 5),
			new InstantCommand(() -> arm.setAngle(position.getArmAngle())),
			new WaitUntilCommand(() -> Math.abs(position.getArmAngle() - arm.getAngle()) < 5),
			new InstantCommand(() -> arm.setExtension(position.getArmExtension())),
			new InstantCommand(() -> intake.setAngle(position.getIntakeAngle())),
			new WaitUntilCommand(() -> Math.abs(position.getArmExtension() - arm.getExtension()) < 5)
		);
	}
}
