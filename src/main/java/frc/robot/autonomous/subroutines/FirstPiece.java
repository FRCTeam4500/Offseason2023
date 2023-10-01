package frc.robot.autonomous.subroutines;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.baseCommands.RumbleCommand;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.utilities.TrajectoryUtilities;

public class FirstPiece extends SequentialCommandGroup{
    public FirstPiece() {
        Trajectory robotTrajectory = TrajectoryUtilities.getDeployedTrajectory("TestPath");
        addCommands(
            new InstantCommand(() -> SwerveDrive.getInstance().resetPose(robotTrajectory.getInitialPose())),
            TrajectoryUtilities.generateSwervePathFollowingCommand(robotTrajectory), 
            new InstantCommand(() -> MessagingSystem.getInstance().addMessage("First Subroutine Done")),
            new RumbleCommand(1));
    }
}
