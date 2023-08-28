package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.EnumConstants.IntakeSpeed;
import frc.robot.Constants.EnumConstants.PlacerState;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.subsystem.placer.arm.Arm;
import frc.robot.subsystem.placer.intake.Intake;

public class AutoPlaceCommand extends SequentialCommandGroup {

	public AutoPlaceCommand(Arm arm, Intake intake, PlacerState state) {
		switch (state) {
			case HighUprightCone:
				addCommands(
					new SetArmAndIntakeCommand(
						arm,
						intake,
						PlacerState.HighUprightCone
					),
					new WaitCommand(1),
					new SetIntakeSpeedCommand(intake, IntakeSpeed.PlaceCone),
					new WaitCommand(1),
					new ZeroCommand(arm, intake)
				);
				break;
			case MidTiltedCone:
				addCommands(
					new SetArmAndIntakeCommand(
						arm,
						intake,
						PlacerState.MidTiltedCone
					),
					new WaitCommand(1),
					new SetIntakeSpeedCommand(intake, IntakeSpeed.PlaceCone),
					new WaitCommand(1),
					new ZeroCommand(arm, intake)
				);
				break;
			case MidUprightCone:
				addCommands(
					new SetArmAndIntakeCommand(ArmPosition.),
					new WaitCommand(1),
					new SetIntakeSpeedCommand(intake, IntakeSpeed.PlaceCone),
					new WaitCommand(1),
					new ZeroCommand(arm, intake)
				);
				break;
			case HighCube:
				addCommands(
					new SetArmAndIntakeCommand(ArmPosition.Top),
					new WaitCommand(1),
					new SetIntakeSpeedCommand(intake, IntakeSpeed.PlaceCube),
					new WaitCommand(1),
					new ZeroCommand(arm, intake)
				);
				break;
			case MidCube:
				addCommands(
					new SetArmAndIntakeCommand(ArmPosition.Mid),
					new WaitCommand(1),
					new SetIntakeSpeedCommand(intake, IntakeSpeed.PlaceCube),
					new WaitCommand(1),
					new ZeroCommand(arm, intake)
				);
				break;
			default:
		}
	}
}
