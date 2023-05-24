package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.EnumConstants.IntakeSpeed;
import frc.robot.Constants.EnumConstants.PlacerState;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.subsystem.placer.arm.Arm;
import frc.robot.subsystem.placer.intake.Intake;

public class AutoPickupCommand extends SequentialCommandGroup {

	public AutoPickupCommand(Arm arm, Intake intake, GamePiece piece) {
		switch (piece) {
			case Cube:
				addCommands(
					new SetArmAndIntakeCommand(
						arm,
						intake,
						PlacerState.GroundPickup
					),
					new SetIntakeSpeedCommand(intake, IntakeSpeed.PickupCube)
				);
				break;
			case TiltedCone:
			case UprightCone:
				addCommands(
					new SetArmAndIntakeCommand(
						arm,
						intake,
						PlacerState.GroundPickup
					),
					new SetIntakeSpeedCommand(
						intake,
						IntakeSpeed.PickupUprightCone
					)
				);
				break;
			case Nothing:
		}
	}
}
