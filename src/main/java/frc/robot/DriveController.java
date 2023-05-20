package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.commands.baseCommands.ResetGyroCommand;
import frc.robot.commands.complexCommands.PlaceCommand;
import frc.robot.commands.complexCommands.SwerveDriveCommand;
import frc.robot.commands.complexCommands.ZeroCommand;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.swerve.SwerveDrive;

public class DriveController extends CommandXboxController {
    Intake intake;
    Arm arm;
    SwerveDrive swerve;
    SwerveDriveCommand swerveCommand;

    private static DriveController controller = null;

    private final Trigger switchDriveModeButton = this.x();
    private final Trigger resetGyroButton = this.a();
    private final Trigger slowModeButton = this.leftBumper();
    private final Trigger driverPlaceButton = this.b();
    
    private DriveController(int Connectedport, Intake intake, Arm arm, SwerveDrive drive, SwerveDriveCommand swerveCommand) {
        super(Connectedport);
        this.intake = intake;
        this.arm = arm;
        this.swerve = drive;
        this.swerveCommand = swerveCommand;

        setSwerveButtons();
    }

    /**
     * Creates a new instance of the controller. If the controller is null, it will create a new one.
     * Should be called first before getIntstance() to ensure that the controller is not null.
     * @param Connectedport
     * @param intake
     * @param arm
     * @param drive
     * @param swerveCommand
     * @return controller type "DriveController"
     */
    public static synchronized DriveController getInstance(int Connectedport, Intake intake, Arm arm, SwerveDrive drive, SwerveDriveCommand swerveCommand) {
        if (controller == null) {
            controller = new DriveController(Connectedport, intake, arm, drive, swerveCommand);
        }
        return controller;
    }

    /**
     * Gets the instance of the controller. If the controller is null, it will return null.
     * @return
     */
    public static synchronized DriveController getInstance() {
        return controller;
    }

    public void setSwerveButtons() {
        swerveCommand = new SwerveDriveCommand(swerve, this);
        swerve.setDefaultCommand(swerveCommand);

        switchDriveModeButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.switchControlMode();}));
        resetGyroButton.toggleOnTrue(new ResetGyroCommand(swerve));
        
        slowModeButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.slowSpeed();}));
        slowModeButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.fastSpeed();}));

        driverPlaceButton.and(() -> Intake.getGamePiece().get() == GamePiece.TiltedCone || 
        Intake.getGamePiece().get() == GamePiece.UprightCone).toggleOnTrue(
            new PlaceCommand(arm, intake, GamePiece.UprightCone)
        );

        driverPlaceButton.and(() -> Intake.getGamePiece().get() == GamePiece.Cube).toggleOnTrue(
            new PlaceCommand(arm, intake, GamePiece.Cube)
        );
        driverPlaceButton.toggleOnFalse(
            new ZeroCommand(arm, intake)
        );
        
        Shuffleboard.getTab("Swerve").add("Swerve", swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
    }

}
