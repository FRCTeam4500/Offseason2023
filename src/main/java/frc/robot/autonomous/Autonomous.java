package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EnumConstants.ArmPosition;
import frc.robot.autonomous.autos.BalanceAuto;
import frc.robot.autonomous.autos.OnePieceAuto;
import frc.robot.subsystems.placer.arm.Arm;
import frc.robot.subsystems.placer.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Autonomous {
	private static Autonomous autonomous = null;
	SwerveDrive swerve;
	Arm arm;
	Intake intake;
	private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();
	private final SendableChooser<ArmPosition> firstPieceHeightChooser = new SendableChooser<ArmPosition>();
	private final SendableChooser<Pose2d> startingPositionChooser = new SendableChooser<Pose2d>();

	private Autonomous() {
		swerve = SwerveDrive.getInstance();
		arm = Arm.getInstance();
		intake = Intake.getInstance();
		configureAuto();
	}

	private void configureAuto() {
		autonChooser.setDefaultOption("No auto", null);
		autonChooser.addOption("One Piece", new OnePieceAuto());
		autonChooser.addOption("Balance", new BalanceAuto());
		Shuffleboard.getTab("Display").add("Auto Route", autonChooser);

		firstPieceHeightChooser.setDefaultOption("Top", ArmPosition.Top);
		firstPieceHeightChooser.addOption("Middle", ArmPosition.Mid);
		Shuffleboard.getTab("Display").add("First Piece Height", firstPieceHeightChooser);

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

	public Command getAutonCommand() {
		return autonChooser.getSelected();
	}

	public ArmPosition getFirstPieceHeight() {
		return firstPieceHeightChooser.getSelected();
	}

	public Pose2d getStartingPosition() {
		return startingPositionChooser.getSelected();
	}
}
