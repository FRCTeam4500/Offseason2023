package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.autonomous.subroutines.BackToGrid;
import frc.robot.autonomous.subroutines.FirstPiece;
import frc.robot.commands.complexCommands.AutoPlaceCommand;
import frc.robot.subsystems.placer.intake.Intake;

public class TestAuto extends SequentialCommandGroup {

	public TestAuto() {
		addCommands(
			new InstantCommand(() -> Intake.setGamePiece(GamePiece.Cone)),
			new AutoPlaceCommand(ArmPosition.Top)
		);
	}
}
