package frc.robot.utilities;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.placer.Placer;
import frc.robot.subsystems.placer.arm.Arm;
import frc.robot.subsystems.placer.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swervydwervy.Swerve;
import frc.robot.subsystems.vision.Vision;

import java.util.TimerTask;
import org.littletonrobotics.junction.Logger;

public class LogSubsystemInputsTask extends TimerTask {

	Arm arm;
	Intake intake;
	Swerve swerve;
	MessagingSystem messagingSystem;
	Vision vision;

	public LogSubsystemInputsTask() {
		this.swerve = Swerve.getInstance();
		this.arm = Arm.getInstance();
		this.intake = Intake.getInstance();
		this.messagingSystem = MessagingSystem.getInstance();
		this.vision = Vision.getInstance();
	}

	@Override
	public void run() {
		arm.updateInputs(arm.getInputs());
		intake.updateInputs(intake.getInputs());
		swerve.updateInputs(swerve.getInputs());
		messagingSystem.updateInputs(messagingSystem.getInputs());
		vision.updateInputs(vision.getInputs());

		Logger.getInstance().processInputs("Arm", arm.getInputs());
		Logger.getInstance().processInputs("Intake", intake.getInputs());
		Logger.getInstance().processInputs("Swerve", swerve.getInputs());
		Logger.getInstance().processInputs("Messaging System", messagingSystem.getInputs());
		Logger.getInstance().processInputs("Vision", vision.getInputs());
		Logger.getInstance().recordOutput("Odometry", swerve.getPoseMeters());
		SwerveModuleState[] states = swerve.getModuleStates();
		Logger
				.getInstance()
				.recordOutput(
						"ModuleStates",
						states[0],
						states[1],
						states[2],
						states[3]);
		Logger
				.getInstance()
				.recordOutput("Placer", Placer.getInstance().getMechanism2d());
	}
}
