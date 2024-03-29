package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.EnumConstants.VisionTarget;
import frc.robot.autonomous.autos.BalanceAuto;
import frc.robot.commands.autoCommands.AutoAlignHorizontalCommand;
import frc.robot.commands.autoCommands.AutoAlignRotationalCommand;
import frc.robot.commands.baseCommands.CancellationCommand;
import frc.robot.commands.baseCommands.ResetGyroCommand;
import frc.robot.commands.complexCommands.AutoBalanceCommand;
import frc.robot.commands.complexCommands.PlaceCommand;
import frc.robot.commands.complexCommands.SwerveDriveCommand;
import frc.robot.commands.complexCommands.TeleopZeroCommand;
import frc.robot.subsystems.swerve.SwerveDrive;

public class DriveController extends CommandXboxController {

	SwerveDriveCommand swerveCommand;

	private static DriveController instanceDriveController = null;

	private final Trigger switchDriveModeButton = this.x();
	private final Trigger resetGyroButton = this.a();
	private final Trigger slowModeButton = this.leftBumper();
	private final Trigger driverPlaceButton = this.b();
	private final Trigger alignGamePieceButton = this.rightBumper();
	private final Trigger alignTapeButton = this.povRight();
	private final Trigger alignTapeButton2 = this.povLeft();
	private final Trigger balanceButton = this.y();
	private final Trigger cancelButton = this.start();

	private DriveController() {
		super(JoystickConstants.DRIVER_PORT);
		setButtons();
	}

	public static synchronized DriveController getInstance() {
		if (instanceDriveController == null) {
			instanceDriveController = new DriveController();
		}
		return instanceDriveController;
	}

	public void setButtons() {
		swerveCommand = new SwerveDriveCommand(this);
		Command balanceCommand = new AutoBalanceCommand();
		SwerveDrive.getInstance().setDefaultCommand(swerveCommand);

		switchDriveModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.switchControlMode()));

		resetGyroButton.toggleOnTrue(new ResetGyroCommand());

		slowModeButton.toggleOnTrue(new InstantCommand(() -> swerveCommand.slowSpeed()));
		slowModeButton.toggleOnFalse(new InstantCommand(() -> swerveCommand.fastSpeed()));

		cancelButton.toggleOnTrue(new CancellationCommand());

		balanceButton.toggleOnTrue(balanceCommand);
		balanceButton.toggleOnFalse(new CancellationCommand());

		driverPlaceButton.toggleOnTrue(
			new PlaceCommand()
			.andThen(new WaitCommand(0.5))
			.andThen(new TeleopZeroCommand())
		);


		alignGamePieceButton.toggleOnTrue(new AutoAlignRotationalCommand(VisionTarget.GamePiece));
		alignTapeButton.toggleOnTrue(new AutoAlignHorizontalCommand(VisionTarget.ReflectiveTape));
		alignTapeButton2.toggleOnTrue(new AutoAlignHorizontalCommand(VisionTarget.ReflectiveTape));
	}
}
