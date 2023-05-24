package frc.robot.utility;

import frc.robot.subsystem.placer.arm.Arm;
import frc.robot.subsystem.placer.intake.Intake;
import frc.robot.subsystem.swerve.SwerveDrive;
import java.util.TimerTask;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class LogSubsystemInputsTask extends TimerTask {

	Arm arm;
	Intake intake;
	SwerveDrive swerve;

	public LogSubsystemInputsTask() {
		this.swerve = SwerveDrive.getInstance();
		this.arm = Arm.getInstance();
		this.intake = Intake.getInstance();
	}

	@Override
	public void run() {
		arm.updateInputs(arm.getInputs());
		intake.updateInputs(intake.getInputs());
		swerve.updateInputs(swerve.getInputs());

		Logger.getInstance().processInputs("Arm", arm.getInputs());
		Logger.getInstance().processInputs("Intake", intake.getInputs());
		Logger.getInstance().processInputs("Swerve", swerve.getInputs());
		Logger.getInstance().recordOutput("Odometry", swerve.getRobotPose());
		SwerveModuleState[] states = swerve.getModuleStates();
		Logger
			.getInstance()
			.recordOutput("ModuleStates", states[0], states[1], states[2], states[3]);
	}
}
