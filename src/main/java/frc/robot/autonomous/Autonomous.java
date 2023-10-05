package frc.robot.autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.placer.arm.Arm;
import frc.robot.subsystems.placer.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Autonomous {

	private static Autonomous autonomous = null;
	SwerveDrive swerve;
	Arm arm;
	Intake intake;
	private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

	private Autonomous() {
		swerve = SwerveDrive.getInstance();
		arm = Arm.getInstance();
		intake = Intake.getInstance();
		configureAuto();
	}

	private void configureAuto() {
		autonChooser.setDefaultOption("No auto", null);
		Shuffleboard.getTab("Auto").add("Auto Routes", autonChooser);
	}

	public static synchronized Autonomous getInstance() {
		if (autonomous == null) {
			autonomous = new Autonomous();
		}
		return autonomous;
	}

	public Command getAutonCommand() {
		return autonChooser.getSelected();
	}
}
