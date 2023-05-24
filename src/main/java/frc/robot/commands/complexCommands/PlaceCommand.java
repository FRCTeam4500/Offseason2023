package frc.robot.commands.complexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.EnumConstants.IntakeSpeed;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.subsystem.placer.arm.Arm;
import frc.robot.subsystem.placer.intake.Intake;

public class PlaceCommand extends SequentialCommandGroup {

	public PlaceCommand(Arm arm, Intake intake, GamePiece piece) {
		switch (piece) {
			case TiltedCone:
			case UprightCone:
				addCommands(
					new SetIntakeSpeedCommand(intake, IntakeSpeed.PlaceCone),
					new WaitCommand(1),
					new SetIntakeSpeedCommand(intake, IntakeSpeed.Off)
				);
				break;
			case Cube:
				addCommands(
					new SetIntakeSpeedCommand(intake, IntakeSpeed.PlaceCube),
					new WaitCommand(1),
					new SetIntakeSpeedCommand(intake, IntakeSpeed.Off)
				);
				break;
			case Nothing:
		}
	}
}
