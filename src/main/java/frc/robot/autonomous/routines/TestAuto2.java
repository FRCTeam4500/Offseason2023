package frc.robot.autonomous.routines;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autoCommands.AutoTimedDriveCommand;

public class TestAuto2 extends SequentialCommandGroup{
    public TestAuto2() {
        addCommands(
            new AutoTimedDriveCommand(new ChassisSpeeds(1, 0, Math.PI/2), 3),
            new WaitCommand(0.5),
            new AutoTimedDriveCommand(new ChassisSpeeds(-1, 0, -Math.PI/2), 3)
        );
    }
}
