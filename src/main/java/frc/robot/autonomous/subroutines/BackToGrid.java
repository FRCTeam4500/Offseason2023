package frc.robot.autonomous.subroutines;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.baseCommands.RumbleCommand;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.utilities.TrajectoryUtilities;

public class BackToGrid extends SequentialCommandGroup{
    public BackToGrid() {
        Trajectory robotTrajectory = TrajectoryUtilities.getDeployedTrajectory("TestPath2");
        addCommands(
            TrajectoryUtilities.generateSwervePathFollowingCommand(robotTrajectory), 
            new InstantCommand(() -> MessagingSystem.getInstance().addMessage("Second Subroutine Done")),
            new RumbleCommand(1)
        );
    }
}
