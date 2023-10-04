package frc.robot.autonomous.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoCommands.AutoTimedDriveCommand;
import frc.robot.commands.complexCommands.AutoBalanceCommand;

public class BalanceAuto extends SequentialCommandGroup{
    public BalanceAuto() {
        addCommands(
            new OnePieceAuto(),
            new AutoTimedDriveCommand(new ChassisSpeeds(1, 0, 0), 1),
            new AutoBalanceCommand()
        );
    }
}
