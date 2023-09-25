package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.commands.complexCommands.SwerveExampleCommand.AbsoluteDrive;
import frc.robot.commands.complexCommands.SwerveExampleCommand.AbsoluteFieldDrive;
import frc.robot.commands.complexCommands.SwerveExampleCommand.TeleopDrive;
import frc.robot.subsystem.*;
import frc.robot.subsystem.swerve.SwerveDrive;
import frc.robot.subsystem.swerve.SwerveExample;
import java.io.File;

public class RobotContainer {

	// private final SwerveDrive swerve = SwerveDrive.getInstance();
	// private final Arm arm = Arm.getInstance();
	// private final Intake intake = Intake.getInstance();

	// /* Setting controller Buttons */
	// private final DriveController driveStick = DriveController.getInstance(JoystickConstants.DRIVER_PORT, intake, arm, swerve);
	// private final OperatorController controlJoystick = OperatorController.getInstance(JoystickConstants.OPERATOR_PORT, intake, arm);

	// private final Autonomous autonomous = Autonomous.getInstance(swerve, arm, intake);

	// The robot's subsystems and commands are defined here...
	private final SwerveExample drivebase = new SwerveExample(
		new File(Filesystem.getDeployDirectory(), "swerve/neo")
	);
	// CommandJoystick rotationController = new CommandJoystick(1);
	// Replace with CommandPS4Controller or CommandJoystick if needed
	CommandJoystick driverController = new CommandJoystick(1);

	// CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
	XboxController driverXbox = new XboxController(0);

	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(
			drivebase,
			// Applies deadbands and inverts controls because joysticks
			// are back-right positive while robot
			// controls are front-left positive
			() ->
				MathUtil.applyDeadband(
					driverXbox.getLeftY(),
					OperatorConstants.LEFT_Y_DEADBAND
				),
			() ->
				MathUtil.applyDeadband(
					driverXbox.getLeftX(),
					OperatorConstants.LEFT_X_DEADBAND
				),
			() -> -driverXbox.getRightX(),
			() -> -driverXbox.getRightY(),
			false
		);

		AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(
			drivebase,
			() ->
				MathUtil.applyDeadband(
					driverXbox.getLeftY(),
					OperatorConstants.LEFT_Y_DEADBAND
				),
			() ->
				MathUtil.applyDeadband(
					driverXbox.getLeftX(),
					OperatorConstants.LEFT_X_DEADBAND
				),
			() -> driverXbox.getRawAxis(2),
			false
		);
		TeleopDrive simClosedFieldRel = new TeleopDrive(
			drivebase,
			() ->
				MathUtil.applyDeadband(
					driverXbox.getLeftY(),
					OperatorConstants.LEFT_Y_DEADBAND
				),
			() ->
				MathUtil.applyDeadband(
					driverXbox.getLeftX(),
					OperatorConstants.LEFT_X_DEADBAND
				),
			() -> driverXbox.getRawAxis(2),
			() -> true,
			false,
			true
		);
		TeleopDrive closedFieldRel = new TeleopDrive(
			drivebase,
			() ->
				MathUtil.applyDeadband(
					driverController.getY(),
					OperatorConstants.LEFT_Y_DEADBAND
				),
			() ->
				MathUtil.applyDeadband(
					driverController.getX(),
					OperatorConstants.LEFT_X_DEADBAND
				),
			() -> -driverController.getRawAxis(3),
			() -> true,
			false,
			true
		);

		drivebase.setDefaultCommand(
			!RobotBase.isSimulation()
				? closedAbsoluteDrive
				: closedFieldAbsoluteDrive
		);
	}

	private void configureBindings() {
		// Schedule `ExampleCommand` when `exampleCondition` changes to `true`

		new JoystickButton(driverXbox, 1)
			.onTrue((new InstantCommand(drivebase::zeroGyro)));
		new JoystickButton(driverXbox, 3)
			.onTrue(new InstantCommand(drivebase::addFakeVisionReading));
		//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.exampleAuto(drivebase);
	}

	public void setDriveMode() {
		//drivebase.setDefaultCommand();
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}

	public void teleopInit() {
		Command auton = autonomous.getAutonCommand();
		if (auton != null) {
			auton.cancel();
		}
	}

	public void disabledInit() {
		swerve.zeroModules();
	}
}
