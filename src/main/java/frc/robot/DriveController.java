package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.autonomous.AutonomousDrivingCommand;
import frc.robot.autonomous.AutonomousDrivingCommand.AutoDriveMode;
import frc.robot.autonomous.AutonomousDrivingCommand.AutoDriveStage;
import frc.robot.commands.baseCommands.CancellationCommand;
import frc.robot.commands.baseCommands.ResetGyroCommand;
import frc.robot.commands.complexCommands.AutoAlignRotationalCommand;
import frc.robot.commands.complexCommands.AutoBalanceCommand;
import frc.robot.commands.complexCommands.AutoDrivetoCommand;
import frc.robot.commands.complexCommands.PlaceCommand;
import frc.robot.commands.complexCommands.SwerveDriveCommand;
import frc.robot.commands.complexCommands.ZeroCommand;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriveController extends CommandXboxController {

	SwerveDriveCommand swerveCommand;
	private AutonomousDrivingCommand autoDriveCommand;

	private static DriveController instanceDriveController = null;

	private final Trigger switchDriveModeButton = this.x();
	private final Trigger resetGyroButton = this.a();
	private final Trigger slowModeButton = this.leftBumper();
	private final Trigger driverPlaceButton = this.b();
	private final Trigger alignButton = this.y();
	private final Trigger cancelButton = this.start();
	private final Trigger balanceButton = this.povUp();
	private final Trigger autoTestButton = this.povRight();

	private DriveController() {
		super(JoystickConstants.DRIVER_PORT);
		setButtons();
		addToShuffleBoard();
	}

	public static synchronized DriveController getInstance() {
		if (instanceDriveController == null) {
			instanceDriveController = new DriveController();
		}
		return instanceDriveController;
	}

	public void setButtons() {
		swerveCommand = new SwerveDriveCommand(this);
		autoDriveCommand = new AutonomousDrivingCommand(new AutoDriveStage(AutoDriveMode.GamePieceAlign, new Pose2d(1, 0, new Rotation2d()), 1));

		SwerveDrive.getInstance().setDefaultCommand(swerveCommand);

		switchDriveModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.switchControlMode()));

		resetGyroButton.toggleOnTrue(new ResetGyroCommand());

		slowModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.slowSpeed()));
		slowModeButton.toggleOnFalse(new InstantCommand(() -> swerveCommand.fastSpeed()));

		alignButton.toggleOnTrue(
			new SequentialCommandGroup(
				new AutoAlignRotationalCommand(0, 1, 2),
				new AutoDrivetoCommand(0),
				new AutoAlignRotationalCommand(0, 1, 2)
			)
		);

		cancelButton.toggleOnTrue(new CancellationCommand());

		balanceButton.toggleOnTrue(new AutoBalanceCommand(1, 5));

		driverPlaceButton.toggleOnTrue(new PlaceCommand());
		driverPlaceButton.toggleOnFalse(new ZeroCommand());

		autoTestButton.toggleOnTrue(autoDriveCommand);
	}
	public void addToShuffleBoard() {
		Shuffleboard.getTab("Messaging").add("Messaging System", MessagingSystem.getInstance());
		Shuffleboard.getTab("Swerve").add("Swerve", SwerveDrive.getInstance());
		Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
		Shuffleboard.getTab("Swerve").add("Auto", autoDriveCommand);
	}
}
