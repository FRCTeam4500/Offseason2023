package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoTurnCommand extends CommandBase{
    private SwerveDrive swerve;
    private PIDController pid;
    private int timeCorrect;
    public AutoTurnCommand(double targetAngle) {
        swerve = SwerveDrive.getInstance();
        pid = new PIDController(targetAngle, targetAngle, targetAngle);
        pid.setSetpoint(Units.degreesToRadians(targetAngle));
        pid.setTolerance(1);
        pid.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        pid.reset();
        timeCorrect = 0;
    }

    @Override
    public void execute() {
        double turnSpeed = 1.5 * pid.calculate(swerve.getRobotAngle());
        swerve.driveRobotCentric(0, 0, turnSpeed);
        if (pid.atSetpoint()) {
            timeCorrect++;
        } else {
            timeCorrect = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return timeCorrect > 50;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveRobotCentric(0, 0, 0);
    }
}
