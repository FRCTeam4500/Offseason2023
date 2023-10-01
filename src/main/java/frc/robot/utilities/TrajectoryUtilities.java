package frc.robot.utilities;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TrajectoryUtilities {
    public static Trajectory getDeployedTrajectory(String trajectoryName) {
        String trajectoryJSON = "output/" + trajectoryName + ".wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory trajectory = null;
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch(IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryName, ex.getStackTrace());
        }
        return trajectory;
    }

    public static SequentialCommandGroup generateSwervePathFollowingCommand(String trajectoryName, Rotation2d endRotation) {
        SwerveDrive swerve = SwerveDrive.getInstance();
        Trajectory path = getDeployedTrajectory(trajectoryName);
        ProfiledPIDController anglePID = new ProfiledPIDController(4, 0, 0, new Constraints(SwerveConstants.MAX_ROTATIONAL_SPEED, SwerveConstants.MAX_ROTATIONAL_ACCELERATION));
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        Supplier<Rotation2d> rotation = () -> endRotation;
        return new SwerveControllerCommand(path, swerve::getRobotPose, swerve.getKinematics(), new PIDController(1, 0, 0), new PIDController(1, 0, 0), anglePID, rotation, swerve::driveModules, swerve)
            .andThen(() -> swerve.driveRobotCentric(0, 0, 0));
    }

    public static SequentialCommandGroup generateSwervePathFollowingCommand(Trajectory trajectory, Rotation2d endRotation) {
        SwerveDrive swerve = SwerveDrive.getInstance();
        ProfiledPIDController anglePID = new ProfiledPIDController(4, 0, 0, new Constraints(SwerveConstants.MAX_ROTATIONAL_SPEED, SwerveConstants.MAX_ROTATIONAL_ACCELERATION));
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        Supplier<Rotation2d> rotation = () -> endRotation;
        return new SwerveControllerCommand(trajectory, swerve::getRobotPose, swerve.getKinematics(), new PIDController(1, 0, 0), new PIDController(1, 0, 0), anglePID, rotation, swerve::driveModules, swerve)
            .andThen(() -> swerve.driveRobotCentric(0, 0, 0));
    }

    public static SequentialCommandGroup generateSwervePathFollowingCommand(Trajectory trajectory) {
        Rotation2d rotation = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation();
        return generateSwervePathFollowingCommand(trajectory, rotation);
    }

    public static SequentialCommandGroup generateSwervePathFollowingCommand(String trajectoryName) {
        Rotation2d rotation = getDeployedTrajectory(trajectoryName).getStates().get(getDeployedTrajectory(trajectoryName).getStates().size() - 1).poseMeters.getRotation();
        return generateSwervePathFollowingCommand(trajectoryName, rotation);
    }
}
