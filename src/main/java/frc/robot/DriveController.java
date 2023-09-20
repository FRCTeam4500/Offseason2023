package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.baseCommands.ResetGyroCommand;
import frc.robot.commands.complexCommands.AutoAlignRotationalCommand;
import frc.robot.commands.complexCommands.AutoDrivetoCommand;
import frc.robot.commands.complexCommands.PlaceCommand;
import frc.robot.commands.complexCommands.SwerveDriveCommand;
import frc.robot.commands.complexCommands.ZeroCommand;
import frc.robot.subsystem.messaging.MessagingSystem;
import frc.robot.subsystem.placer.arm.Arm;
import frc.robot.subsystem.placer.intake.Intake;
import frc.robot.subsystem.swerve.SwerveDrive;

public class DriveController extends CommandXboxController {

	Intake intake;
	Arm arm;
	MessagingSystem messagingSystem;
	SwerveDrive swerve;
	SwerveDriveCommand swerveCommand;

	private static DriveController instanceDriveController = null;

	private final Trigger switchDriveModeButton = this.x();
	private final Trigger resetGyroButton = this.a();
	private final Trigger slowModeButton = this.leftBumper();
	private final Trigger driverPlaceButton = this.b();
	private final Trigger alignButtion = this.y();

	private DriveController() {
		super(JoystickConstants.DRIVER_PORT);
		intake = Intake.getInstance();
		arm = Arm.getInstance();
		messagingSystem = MessagingSystem.getInstance();
		swerve = SwerveDrive.getInstance();
		swerveCommand = new SwerveDriveCommand(swerve, this);

		setSwerveButtons();
	}

	/**
	 * Creates a new instance of the controller. If the controller is null, it will create a new one.
	 * Should be called first before getIntstance() to ensure that the controller is not null.
	 * @return controller type "DriveController"
	 */
	public static synchronized DriveController getInstance() {
		if (instanceDriveController == null) {
			instanceDriveController =
				new DriveController();
		}
		return instanceDriveController;
	}

	public void setSwerveButtons() {
		swerveCommand = new SwerveDriveCommand(swerve, this);
		swerve.setDefaultCommand(swerveCommand);

		switchDriveModeButton.toggleOnTrue(
			new InstantCommand(() -> {
				swerveCommand.switchControlMode();
			})
		);
		resetGyroButton.toggleOnTrue(new ResetGyroCommand(swerve));

		slowModeButton.toggleOnTrue(
			new InstantCommand(() -> {
				swerveCommand.slowSpeed();
			})
		);
		slowModeButton.toggleOnFalse(
			new InstantCommand(() -> {
				swerveCommand.fastSpeed();
			})
		);

		alignButtion.toggleOnTrue(
			new SequentialCommandGroup(
				new AutoAlignRotationalCommand(0, 1, 2),
				new AutoDrivetoCommand(0),
				new AutoAlignRotationalCommand(0, 1, 2)
			)
		);

		driverPlaceButton.toggleOnTrue(new PlaceCommand());

		driverPlaceButton.toggleOnFalse(new ZeroCommand());

		Shuffleboard.getTab("Messaging").add("Messaging System", messagingSystem);
		Shuffleboard.getTab("Swerve").add("Swerve", swerve);
		Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
	}
}
