package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.EnumConstants.ControlMode;
import frc.robot.Constants.EnumConstants.VisionTarget;
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
	private final Trigger cancelButton = this.start();
	private final Trigger aprilTagAlignButton = this.povLeft();
	private final Trigger gamePieceAlignButton = this.povRight();
	private final Trigger reflectiveTapeAlignButton = this.povDown();

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
		swerveCommand = SwerveDriveCommand.getInstance(this);
		SwerveDrive.getInstance().setDefaultCommand(swerveCommand);

		switchDriveModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.switchControlMode()));

		resetGyroButton.toggleOnTrue(new ResetGyroCommand());

		slowModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.slowSpeed()));
		slowModeButton.toggleOnFalse(new InstantCommand(() -> swerveCommand.fastSpeed()));

		cancelButton.toggleOnTrue(new CancellationCommand());

		driverPlaceButton.toggleOnTrue(new PlaceCommand());
		driverPlaceButton.toggleOnFalse(new ZeroCommand());

		aprilTagAlignButton.toggleOnTrue(
			new InstantCommand(() -> {
				swerveCommand.setVisionTarget(VisionTarget.AprilTag);
				swerveCommand.setControlMode(ControlMode.AlignToTarget);
			})
		);

		reflectiveTapeAlignButton.toggleOnTrue(
			new InstantCommand(() -> {
				swerveCommand.setVisionTarget(VisionTarget.ReflectiveTape);
				swerveCommand.setControlMode(ControlMode.AlignToTarget);
			})
		);

		gamePieceAlignButton.toggleOnTrue(
			new InstantCommand(() -> {
				swerveCommand.setVisionTarget(VisionTarget.GamePiece);
				swerveCommand.setControlMode(ControlMode.AimToTarget);
			})
		);
	}

	public void addToShuffleBoard() {
		Shuffleboard.getTab("Messaging").add("Messaging System", MessagingSystem.getInstance());
		Shuffleboard.getTab("Swerve").add("Swerve", SwerveDrive.getInstance());
		Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
	}
}
