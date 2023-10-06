package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.placer.arm.Arm;
import frc.robot.subsystems.placer.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Autonomous {
	private static Autonomous autonomous = null;
	SwerveDrive swerve;
	Arm arm;
	Intake intake;
	private final SendableChooser<String> autonChooser = new SendableChooser<String>();
	private final SendableChooser<Pose2d> startingPositionChooser = new SendableChooser<Pose2d>();

	private Autonomous() {
		swerve = SwerveDrive.getInstance();
		arm = Arm.getInstance();
		intake = Intake.getInstance();
		configureAuto();
	}

	private void configureAuto() {
		autonChooser.setDefaultOption("No auto", null);
		autonChooser.addOption("One Piece", "One Piece");
		autonChooser.addOption("Balance", "Balance");
		Shuffleboard.getTab("Display").add("Auto Route", autonChooser);

		Pose2d midPose = new Pose2d(3.29, 1.9, new Rotation2d(Math.PI));
		Pose2d bumpPose = new Pose2d(0.43, 1.9, new Rotation2d(Math.PI));
		Pose2d sidePose = new Pose2d(5.1, 1.9, new Rotation2d(Math.PI));
		if (DriverStation.getAlliance() == Alliance.Red) {
			midPose = new Pose2d(8 - midPose.getX(), 1.9, new Rotation2d(Math.PI));
			bumpPose = new Pose2d(8 - sidePose.getX(), 1.9, new Rotation2d(Math.PI));
			sidePose = new Pose2d(8 - bumpPose.getX(), 1.9, new Rotation2d(Math.PI));
		}
		startingPositionChooser.setDefaultOption("Middle", midPose);
		startingPositionChooser.addOption("Bump Side", bumpPose);
		startingPositionChooser.addOption("Other Side", sidePose);
		Shuffleboard.getTab("Display").add("Starting Position", startingPositionChooser);
	}

	public static synchronized Autonomous getInstance() {
		if (autonomous == null) {
			autonomous = new Autonomous();
		}
		return autonomous;
	}

	public String getAutonCommand() {
		return autonChooser.getSelected();
	}

	public Pose2d getStartingPosition() {
		return startingPositionChooser.getSelected();
	}
}
