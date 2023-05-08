package frc.robot.commands.complexCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.swerve.SwerveDrive;

public class AutomatedDriveCommand extends CommandBase{
    private SwerveDrive swerve;
    private Pose2d currentPose;
    private Pose2d targetPose;
    private Pose2d differencePose;
    private double translationalThreshold;
    private double rotationalThreshold;
    private double timeThreshold;
    private double timeCorrect = 0;
    private PIDController forwardVelocityController = new PIDController(1, 0, 0);
    private PIDController sidewaysVelocityController = new PIDController(1, 0, 0);
    private PIDController rotationalVelocityController = new PIDController(1, 0, 0);

    /**
     * Drives the robot to a target field-centric position
     * @param swerve the drivetrain
     * @param targetPose the position the robot should go to
     * @param translationalThreshold the tolerance, in meters, for if the robot considers itself to be in the same spot as the target pose
     * @param rotationalThreshold the tolerance, in meters, for if the robot considers itself to be facing the same direction as the target pose
     * @param timeThreshold the time, in seconds, that the robot must be at the target pose
     */
    public AutomatedDriveCommand(SwerveDrive swerve, Pose2d targetPose, double translationalThreshold, double rotationalThreshold, double timeThreshold) {
        this.swerve = swerve;
        this.targetPose = targetPose;
        this.translationalThreshold = translationalThreshold;
        this.rotationalThreshold = rotationalThreshold;
        this.timeThreshold = timeThreshold;
        currentPose = swerve.getRobotPose();
        addRequirements(swerve);
    }

    /**
     * Drives the robot to a target field-centric position
     * @param swerve the drivetrain
     * @param currentPose the current position of the robot. Can be changed for testing purposes
     * @param targetPose the position the robot should go to
     * @param translationalThreshold the tolerance, in meters, for if the robot considers itself to be in the same spot as the target pose
     * @param rotationalThreshold the tolerance, in meters, for if the robot considers itself to be facing the same direction as the target pose
     * @param timeThreshold the time, in seconds, that the robot must be at the target pose
     */
    public AutomatedDriveCommand(SwerveDrive swerve, Pose2d currentPose, Pose2d targetPose, double translationalThreshold, double rotationalThreshold, double timeThreshold) {
        this(swerve, targetPose, translationalThreshold, rotationalThreshold, timeThreshold);
        currentPose = new Pose2d();
    }

    @Override
    public void execute() { 
        double forwardVelocity = forwardVelocityController.calculate(currentPose.getX(), targetPose.getX());
        double sidewaysVelocity = sidewaysVelocityController.calculate(currentPose.getY(), targetPose.getY());
        double rotationalVelocity = rotationalVelocityController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        driveFieldCentric(forwardVelocity, sidewaysVelocity, rotationalVelocity);

        differencePose = targetPose.relativeTo(currentPose);
        if (Math.abs(differencePose.getX()) < translationalThreshold &&
        Math.abs(differencePose.getY()) < translationalThreshold &&
        Math.abs(differencePose.getRotation().getRadians()) < rotationalThreshold) {
            timeCorrect++;
        } else {
            timeCorrect = 0;
        }
    }

    private void driveFieldCentric(double forwardVelocity, double sidewaysVelocity, double rotationalVelocity) {
        swerve.driveFieldCentric(forwardVelocity, sidewaysVelocity, rotationalVelocity);
    }

    @Override
    public boolean isFinished() {
        return timeCorrect / 50 > timeThreshold;
    }
}
