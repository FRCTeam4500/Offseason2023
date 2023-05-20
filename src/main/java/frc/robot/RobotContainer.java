package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.*;
import frc.robot.subsystem.swerve.SwerveDrive;
import frc.robot.Constants.*;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.EnumConstants.IntakeSpeed;
import frc.robot.Constants.EnumConstants.PlacerState;
import frc.robot.commands.baseCommands.*;
import frc.robot.commands.complexCommands.*;
import frc.robot.commands.complexCommands.AutomatedDriveCommand.AutoDriveMode;
import frc.robot.commands.debugCommands.TiltIntakeCommand;


public class RobotContainer {

    private final SwerveDrive swerve = new SwerveDrive();
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();

    private SwerveDriveCommand swerveCommand;

    /** A hash map containing the commands the robot will use in auto <p> These commands can be accessed by putting the cooresponding string key into the .get() method
     * <p> Example: {@code autoCommandMap.get("zero");} 
     */
    public static final HashMap<String, Command> autoCommandMap = new HashMap<>();
    
    /**Both PID constants need to be tested */
    private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerve::getRobotPose, swerve::resetPose, new PIDConstants(5, 0, 0), new PIDConstants(4, 0, 0), swerve::driveModules, autoCommandMap, swerve);    
    private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

    /* Setting Joystick Buttons */
    private final DriveController driveStick = DriveController.getInstance(JoystickConstants.DRIVER_PORT, intake, arm, swerve, swerveCommand);
    private final OperatorController controlJoystick = OperatorController.getInstance(JoystickConstants.OPERATOR_PORT, intake, arm);

    public RobotContainer() {
        configureSwerve();
        configureAuto();
        configureArmAndIntake();
    }
    
    void configureSwerve() {
        Shuffleboard.getTab("Swerve").add("Swerve", swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
    }

    void configureArmAndIntake() {
        Shuffleboard.getTab("Arm and Intake").add("Intake", intake);
        Shuffleboard.getTab("Arm and Intake").add("Arm", arm);
        Shuffleboard.getTab("Arm and Intake").addString("Current Game Piece", () -> Intake.getGamePiece().get().name());
    }

    void configureAuto() {
        autoCommandMap.put(
            "start", 
            new SetArmAndIntakeCommand(arm, intake, PlacerState.Start));

        autoCommandMap.put(
            "zero", 
            new ZeroCommand(arm, intake)
        );

        autoCommandMap.put(
            "placeCubeTop", 
            new AutoPlaceCommand(arm, intake, PlacerState.HighCube)
        );

        autoCommandMap.put(
            "placeConeTop", 
            new AutoPlaceCommand(arm, intake, PlacerState.HighUprightCone)
        );

        autoCommandMap.put(
            "placeCubeMid", 
            new AutoPlaceCommand(arm, intake, PlacerState.MidCube)
        );

        autoCommandMap.put(
            "placeConeMid", 
            new AutoPlaceCommand(arm, intake, PlacerState.MidUprightCone)
        );

        autoCommandMap.put(
            "resetGyro", 
            new ResetGyroCommand(swerve)
        );

        autoCommandMap.put(
            "reverseGyro", 
            new ResetGyroCommand(swerve, 180)
        );

        autoCommandMap.put(
            "pickupCone", 
            new AutoPickupCommand(arm, intake, GamePiece.UprightCone)
        );

        autoCommandMap.put(
            "pickupCube", 
            new AutoPickupCommand(arm, intake, GamePiece.Cube)
        );

        autonChooser.setDefaultOption("Auto Driving Test", new AutomatedDriveCommand(swerve, AutoDriveMode.kRelative, 0.1, 0.1, 1, new Pose2d(2, 0, new Rotation2d()), new Pose2d(0, 0, new Rotation2d())));
        autonChooser.addOption("Blue Bottom: 2 Piece Top", autoBuilder.fullAuto(AutoConstants.BlueBotRedTop2PieceTopAuto));
        autonChooser.addOption("Red Top: 2 Piece Top", autoBuilder.fullAuto(AutoConstants.BlueBotRedTop2PieceTopAuto));
        autonChooser.addOption("Blue Top: 2 Piece Top", autoBuilder.fullAuto(AutoConstants.BlueTopRedBot2PieceTopAuto));
        autonChooser.addOption("Red Bottom: 2 Piece Top", autoBuilder.fullAuto(AutoConstants.BlueTopRedBot2PieceTopAuto));
        autonChooser.addOption("Blue Bottom: 2 Piece Mid", autoBuilder.fullAuto(AutoConstants.BlueBotRedTop2PieceMidAuto));
        autonChooser.addOption("Red Top: 2 Piece Mid", autoBuilder.fullAuto(AutoConstants.BlueBotRedTop2PieceMidAuto));
        autonChooser.addOption("Blue Top: 2 Piece Mid", autoBuilder.fullAuto(AutoConstants.BlueTopRedBot2PieceMidAuto));
        autonChooser.addOption("Red Bottom: 2 Piece Mid", autoBuilder.fullAuto(AutoConstants.BlueTopRedBot2PieceMidAuto));
        autonChooser.addOption("Both Middle: Place, Mobility, and Dock", autoBuilder.fullAuto(AutoConstants.MidPlaceAndDockAuto));
        autonChooser.addOption("Both Side: Place 1 and Mobility", autoBuilder.fullAuto(AutoConstants.PlaceAndMoveAuto));
        autonChooser.addOption("Red Top: Place and Face Substation", autoBuilder.fullAuto(AutoConstants.RedTopPlaceAndRunAuto));
        autonChooser.addOption("Blue Top: Place and Face Substation", autoBuilder.fullAuto(AutoConstants.BlueTopPlaceAndRunAuto));
        Shuffleboard.getTab("Auto").add("Auto Routes", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public void teleopInit() {
        Command auton = autonChooser.getSelected();
        if (auton != null){
            auton.cancel();
        }
    }

    public void disabledInit() {
        swerve.zeroModules();
    }
}