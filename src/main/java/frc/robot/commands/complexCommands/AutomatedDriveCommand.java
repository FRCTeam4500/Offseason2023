package frc.robot.commands.complexCommands;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.swerve.SwerveDrive;
import frc.robot.utility.ExtendedMath;

public class AutomatedDriveCommand extends CommandBase {
    private SwerveDrive swerve;
    private Pose2d[] targetPoses;
    private Pose2d currentTargetPose;
    private Transform2d relativeRobotPose;
    private Pose2d relativeOrigin;
    private ChassisSpeeds currentRobotSpeeds = new ChassisSpeeds();
    private double translationalThreshold;
    private double rotationalThreshold;
    private double timeThreshold;
    private double timeCorrect;
    private int poseCounter;
    private static int timesRun = 0;
    private ComplexWidget thisWidget;
    private AutoDriveMode mode;
    private PIDController forwardVelocityController = new PIDController(5, 0, 0);
    private PIDController sidewaysVelocityController = new PIDController(5, 0, 0);
    private PIDController rotationalVelocityController = new PIDController(2, 0, 0);

    /**
     * Drives the robot to a target field-centric position
     * @param swerve the drivetrain subsystem
     * @param mode the reference frame of the target positions.
     * @param translationalThreshold the tolerance, in meters, for if the robot considers itself to be in the same spot as the target pose
     * @param rotationalThreshold the tolerance, in meters, for if the robot considers itself to be facing the same direction as the target pose
     * @param timeThreshold the time, in seconds, that the robot must be at the target pose before it is considered finished
     * @param targetPose2ds the positions the robot should go to, in the order the robot should go to them 
     */
    public AutomatedDriveCommand(SwerveDrive swerve, AutoDriveMode mode, double translationalThreshold, double rotationalThreshold, double timeThreshold, Pose2d... targetPose2ds) {
        this.swerve = swerve;
        this.targetPoses = targetPose2ds;
        this.translationalThreshold = translationalThreshold;
        this.rotationalThreshold = rotationalThreshold;
        this.timeThreshold = timeThreshold;
        this.mode = mode;
        addRequirements(swerve);
    }

    public enum AutoDriveMode {
        /** The target positions are relative to the robot */
        kRelative,
        /** The target positions are relative to the field */
        kAbsolute
    }

    @Override
    public void initialize() {
        timesRun++;
        thisWidget = Shuffleboard.getTab("Commands").getLayout("Command Grid", BuiltInLayouts.kList).add("Auto Drive Command #" + timesRun, this);
        timeCorrect = 0;
        poseCounter = 0;
        forwardVelocityController.reset();
        sidewaysVelocityController.reset();
        rotationalVelocityController.reset();
        relativeOrigin = swerve.getRobotPose();
        // This next line just converts all the target positions to be relative to the robot, if they aren't already
        if (mode == AutoDriveMode.kAbsolute) {
            for (int i = 0; i < targetPoses.length; i++) {
                targetPoses[i] = targetPoses[i].relativeTo(relativeOrigin);
            }
        }
    }

    @Override
    public void execute() {
        relativeRobotPose = swerve.getRobotPose().minus(relativeOrigin);
        currentTargetPose = targetPoses[poseCounter];
        double forwardVelocity = forwardVelocityController.calculate(relativeRobotPose.getX(), currentTargetPose.getX());
        double sidewaysVelocity = sidewaysVelocityController.calculate(relativeRobotPose.getY(), currentTargetPose.getY());
        double rotationalVelocity = rotationalVelocityController.calculate(relativeRobotPose.getRotation().getRadians() % (2 * Math.PI), currentTargetPose.getRotation().getRadians());
        currentRobotSpeeds = new ChassisSpeeds(forwardVelocity, sidewaysVelocity, rotationalVelocity);
        swerve.driveFieldCentric(forwardVelocity, sidewaysVelocity, rotationalVelocity);

        Pose2d robotPoseRel = new Pose2d(relativeRobotPose.getTranslation(), relativeRobotPose.getRotation());
        if (ExtendedMath.isClose(currentTargetPose, robotPoseRel, translationalThreshold, rotationalThreshold)) {
            timeCorrect++;
        } else {
            timeCorrect = 0;
        }

        if (timeCorrect / 50 >= timeThreshold) { // the / 50 part is because there are 50 scheduler ticks a second
            poseCounter++;
            timeCorrect = 0;
            forwardVelocityController.reset();
            sidewaysVelocityController.reset();
            rotationalVelocityController.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return poseCounter > targetPoses.length - 1;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveFieldCentric(0, 0, 0);
        thisWidget.withProperties(Map.of("Label position", "HIDDEN"));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addIntegerProperty("Position Number: ", () -> poseCounter + 1, null);
        builder.addDoubleProperty("Robot Relative X: ", () -> relativeRobotPose.getX(), null);
        builder.addDoubleProperty("Robot Relative Y: ", () -> relativeRobotPose.getY(), null);
        builder.addDoubleProperty("Robot Relative Rotation: ", () -> relativeRobotPose.getRotation().getRadians(), null);
        builder.addDoubleProperty("Robot Forward Speed: ", () -> currentRobotSpeeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Robot Sideways Speed: ", () -> currentRobotSpeeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("Robot Rotational Speed: ", () -> currentRobotSpeeds.omegaRadiansPerSecond, null);
    }
}
