package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.EnumConstants.VisionTarget;
import frc.robot.commands.autoCommands.AutoAlignCommand;
import frc.robot.commands.autoCommands.AutoAlignHorizontalCommand;
import frc.robot.commands.autoCommands.AutoAlignRotationalCommand;
import frc.robot.commands.baseCommands.CancellationCommand;
import frc.robot.commands.baseCommands.ResetGyroCommand;
import frc.robot.commands.complexCommands.PlaceCommand;
import frc.robot.commands.complexCommands.SwerveDriveCommand;
import frc.robot.commands.complexCommands.ZeroCommand;
import frc.robot.subsystems.messaging.MessagingSystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriveController extends CommandXboxController {

	SwerveDriveCommand swerveCommand;

	private static DriveController instanceDriveController = null;

	private final Trigger switchDriveModeButton = this.x();
	private final Trigger resetGyroButton = this.a();
	private final Trigger slowModeButton = this.leftBumper();
	private final Trigger driverPlaceButton = this.b();
	private final Trigger gamePieceAlignButton = this.y();
	private final Trigger substationAlignButton = this.povUp();
	private final Trigger placeAlignButton = this.povDown();
	private final Trigger cancelButton = this.start();

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
		SwerveDrive.getInstance().setDefaultCommand(swerveCommand);

		switchDriveModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.switchControlMode()));

		resetGyroButton.toggleOnTrue(new ResetGyroCommand());

		slowModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.slowSpeed()));
		slowModeButton.toggleOnFalse(new InstantCommand(() -> swerveCommand.fastSpeed()));

		cancelButton.toggleOnTrue(new CancellationCommand());

		driverPlaceButton.toggleOnTrue(new PlaceCommand());
		driverPlaceButton.toggleOnFalse(new ZeroCommand());

		gamePieceAlignButton.toggleOnTrue(new AutoAlignCommand(VisionTarget.GamePiece));
		placeAlignButton.toggleOnTrue(new AutoAlignCommand(VisionTarget.ReflectiveTape));
		substationAlignButton.toggleOnTrue(new AutoAlignCommand(VisionTarget.AprilTag));
	}

	public void addToShuffleBoard() {
		Shuffleboard.getTab("Messaging").add("Messaging System", MessagingSystem.getInstance());
		Shuffleboard.getTab("Swerve").add("Swerve", SwerveDrive.getInstance());
		Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
	}
}
