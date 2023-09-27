package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.utilities.HelperMethods;

public class AutonomousDrivingCommand extends CommandBase{
    private SwerveDrive swerve;
    private AutoDriveStage stage;
    private PIDController forwardPID;
    private PIDController sidewaysPID;
    private PIDController rotationalPID;
    private double forwardVel;
    private double sidewaysVel;
    private double rotationalVel;
    private Pose2d targetRelativePose;
    private double speedMultiplier;
    private double timesCorrect;
    public AutonomousDrivingCommand(AutoDriveStage stage) {
        this.swerve = SwerveDrive.getInstance();
        this.stage = stage;
        forwardPID = new PIDController(1, 0, 0);
        sidewaysPID = new PIDController(1, 0, 0);
        rotationalPID = new PIDController(1, 0, 0);
        speedMultiplier = (stage.getMaxSpeed() / SwerveConstants.MAX_LINEAR_SPEED) % 1.0;
        forwardPID.reset();
        sidewaysPID.reset();
        rotationalPID.reset();
        forwardPID.setSetpoint(0);
        sidewaysPID.setSetpoint(0);
        rotationalPID.setSetpoint(0);
        forwardVel = 0;
        sidewaysVel = 0;
        rotationalVel = 0;
        targetRelativePose = new Pose2d();
        timesCorrect = 0;
    }

    public void execute() {
        switch (stage.driveMode) {
            default:
            case RelativePoseAlign:
                targetRelativePose = stage.offsetPose.relativeTo(swerve.getRobotPose().relativeTo(stage.originPose));
                forwardVel = speedMultiplier * Math.abs(forwardPID.calculate(targetRelativePose.getX()));
                sidewaysVel = speedMultiplier * Math.abs(sidewaysPID.calculate(targetRelativePose.getY()));
                rotationalVel = speedMultiplier * Math.abs(rotationalPID.calculate(targetRelativePose.getRotation().getRadians()));
                swerve.driveModules(new ChassisSpeeds(forwardVel, sidewaysVel, rotationalVel));     
                if(HelperMethods.isClose(targetRelativePose, new Pose2d(), 0.25, Math.PI/4)) {
                    timesCorrect++;
                } else {
                    timesCorrect = 0;
                }
        }
    }

    public boolean isFinished() {
        return timesCorrect > 50;
    }

    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Forward Vel", () -> forwardVel, null);
        builder.addDoubleProperty("Sideways Vel", () -> sidewaysVel, null);
        builder.addDoubleProperty("Rotational Vel", () -> rotationalVel, null);
        builder.addDoubleProperty("Target X", () -> targetRelativePose.getX(), null);
        builder.addDoubleProperty("Target Y", () -> targetRelativePose.getY(), null);
        builder.addDoubleProperty("Target Z", () -> targetRelativePose.getRotation().getDegrees(), null);

    }

    public enum AutoDriveMode {
        AprilTagAlign,
        GamePieceAlign,
        RelativePoseAlign
    }

    public static class AutoDriveStage {
        private AutoDriveMode driveMode;
        private Pose2d offsetPose;
        private Pose2d originPose;
        private double maxSpeed;
        public AutoDriveStage(AutoDriveMode driveMode, Pose2d offsetPose, double maxSpeed) {
            this.maxSpeed = maxSpeed;
            this.driveMode = driveMode;
            this.offsetPose = offsetPose;
            this.originPose = SwerveDrive.getInstance().getRobotPose();
        }
        public double getMaxSpeed() {return maxSpeed;}
        public AutoDriveMode getDriveMode() {return driveMode;}
        public Pose2d getOffsetPose() {return offsetPose;}
        public Pose2d getOriginPose() {return originPose;}
    }
}
