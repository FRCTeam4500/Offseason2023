package frc.robot.commands.complexCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.swerve.SwerveDrive;

public class AutomatedDriveCommand extends CommandBase{
    private SwerveDrive swerve;
    private Pose2d currentRelativePose;
    private Pose2d targetPose;
    private Pose2d differencePose;
    private Pose2d startPose;
    private double translationalThreshold;
    private double rotationalThreshold;
    private double timeThreshold;
    private double timeCorrect = 0;
    private AutoDriveMode mode;
    private PIDController forwardVelocityController = new PIDController(1, 0, 0);
    private PIDController sidewaysVelocityController = new PIDController(1, 0, 0);
    private PIDController rotationalVelocityController = new PIDController(1, 0, 0);

    /**
     * Drives the robot to a target field-centric position
     * @param swerve the drivetrain subsystem
     * @param currentRelativePose the current position of the robot. Can be changed for testing purposes
     * @param targetPose the position the robot should go to
     * @param translationalThreshold the tolerance, in meters, for if the robot considers itself to be in the same spot as the target pose
     * @param rotationalThreshold the tolerance, in meters, for if the robot considers itself to be facing the same direction as the target pose
     * @param timeThreshold the time, in seconds, that the robot must be at the target pose
     */
    public AutomatedDriveCommand(SwerveDrive swerve, AutoDriveMode mode, Pose2d targetPose, double translationalThreshold, double rotationalThreshold, double timeThreshold) {
        this.swerve = swerve;
        this.targetPose = targetPose;
        this.translationalThreshold = translationalThreshold;
        this.rotationalThreshold = rotationalThreshold;
        this.timeThreshold = timeThreshold;
        this.mode = mode;
        addRequirements(swerve);
    }

    public enum AutoDriveMode {
        kRelative,
        kAbsolute
    }

    @Override
    public void initialize() {
        switch (mode) {
            case kRelative:
                startPose = swerve.getRobotPose();
                currentRelativePose = new Pose2d();
                break;
            case kAbsolute:
                currentRelativePose = swerve.getRobotPose();
        }
    }

    @Override
    public void execute() {
        currentRelativePose = swerve.getRobotPose(); 

        double forwardVelocity = forwardVelocityController.calculate(currentRelativePose.getX(), targetPose.getX());
        double sidewaysVelocity = sidewaysVelocityController.calculate(currentRelativePose.getY(), targetPose.getY());
        double rotationalVelocity = rotationalVelocityController.calculate(currentRelativePose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        swerve.driveFieldCentric(forwardVelocity, sidewaysVelocity, rotationalVelocity);

        differencePose = targetPose.relativeTo(currentRelativePose);
        if (Math.abs(differencePose.getX()) < translationalThreshold &&
        Math.abs(differencePose.getY()) < translationalThreshold &&
        Math.abs(differencePose.getRotation().getRadians()) < rotationalThreshold) {
            timeCorrect++;
        } else {
            timeCorrect = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return timeCorrect / 50 > timeThreshold;
    }

    @Override 
    public void end(boolean interrupted) {
        swerve.driveFieldCentric(0, 0, 0);
    }
}
