package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.subroutines.BackToGrid;
import frc.robot.autonomous.subroutines.FirstPiece;

public class TestAuto extends SequentialCommandGroup{
    public TestAuto() {
        addCommands(
            new FirstPiece(),
            new BackToGrid()
        );
    }
}
