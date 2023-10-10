package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.EnumConstants.VisionTarget;
import frc.robot.Constants.JoystickConstants;
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

	private double modifyJoystickAxis(double joystick, double fineTuneAxis) {
		return (
			-(Math.abs(Math.pow(joystick, 2)) + 0.05) *
			Math.signum(joystick) *
			(1 - (0.5 * fineTuneAxis)) *
			(1 - (0.5 * (this.leftBumper().getAsBoolean() ? 0.4 : 0)))
		);
	}

	double lastHeadingSnapAngle = 0;

	public void setButtons() {
		// swerveCommand = new SwerveDriveCommand(this);
		Command balanceCommand = new AutoBalanceCommand();

		/* Reg field centric drive, open loop */
		SwerveDrive
			.getInstance()
			.setDefaultCommand(
				SwerveDrive
					.getInstance()
					.driveCommand(
						() ->
							modifyJoystickAxis(
								this.getLeftY(),
								this.getLeftTriggerAxis()
							),
						() ->
							modifyJoystickAxis(
								this.getLeftX(),
								this.getLeftTriggerAxis()
							),
						() ->
							modifyJoystickAxis(
								this.getRightX(),
								this.getLeftTriggerAxis()
							),
						true,
						true,
						true
					)
			);

		/* While right trigger, heading align */
		this.rightTrigger()
			.whileTrue(
				SwerveDrive
					.getInstance()
					.headingLockDriveCommand(
						// Use normal translation
						() ->
							modifyJoystickAxis(
								this.getLeftY(),
								this.getLeftTriggerAxis()
							),
						() ->
							modifyJoystickAxis(
								this.getLeftX(),
								this.getLeftTriggerAxis()
							),
						() -> {
							// Find rotation based on 3 "zones" on controller
							var joystickRotation = new Rotation2d(
								this.getRightX(),
								-this.getRightY()
							);
							if (
								Math.sqrt(
									(this.getRightX() * this.getRightX()) +
									(this.getRightY() * this.getRightY())
								) <
								0.2
							) {
								return lastHeadingSnapAngle;
							}

							double correctedJoystickRot = joystickRotation.getRotations() >
								0
								? joystickRotation.getRotations()
								: 1.0 + joystickRotation.getRotations();

							if (
								correctedJoystickRot < 0.917 &&
								correctedJoystickRot > 0.583
							) {
								lastHeadingSnapAngle = Math.PI;
							} else if (
								correctedJoystickRot < 0.583 &&
								correctedJoystickRot > 0.25
							) {
								lastHeadingSnapAngle = Math.PI / 2;
							} else if (
								correctedJoystickRot < 0.25 ||
								correctedJoystickRot > 0.917
							) {
								lastHeadingSnapAngle = Math.PI + (Math.PI / 2);
							}

							return lastHeadingSnapAngle;
						},
						true,
						true
					)
			);

		/* While left trigger, drive closed loop */
		this.leftTrigger()
			.whileTrue(
				SwerveDrive
					.getInstance()
					.driveCommand(
						() ->
							modifyJoystickAxis(
								this.getLeftY(),
								this.getLeftTriggerAxis()
							),
						() ->
							modifyJoystickAxis(
								this.getLeftX(),
								this.getLeftTriggerAxis()
							),
						() ->
							modifyJoystickAxis(
								this.getRightX(),
								this.getLeftTriggerAxis()
							),
						true,
						false,
						true
					)
			);

		switchDriveModeButton.toggleOnTrue(
			new InstantCommand(() -> swerveCommand.switchControlMode())
		);

		resetGyroButton.toggleOnTrue(new ResetGyroCommand());

		slowModeButton.toggleOnTrue(
			new InstantCommand(() -> swerveCommand.slowSpeed())
		);
		slowModeButton.toggleOnFalse(
			new InstantCommand(() -> swerveCommand.fastSpeed())
		);

		cancelButton.toggleOnTrue(new CancellationCommand());

		balanceButton.toggleOnTrue(balanceCommand);
		balanceButton.toggleOnFalse(new CancellationCommand());

		driverPlaceButton.toggleOnTrue(
			new PlaceCommand()
				.andThen(new WaitCommand(0.5))
				.andThen(new TeleopZeroCommand())
		);

		alignGamePieceButton.toggleOnTrue(
			new AutoAlignRotationalCommand(VisionTarget.GamePiece)
		);
		alignTapeButton.toggleOnTrue(
			new AutoAlignHorizontalCommand(VisionTarget.ReflectiveTape)
		);
		alignTapeButton2.toggleOnTrue(
			new AutoAlignHorizontalCommand(VisionTarget.ReflectiveTape)
		);
	}
}
