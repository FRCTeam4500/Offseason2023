package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystem.*;
import frc.robot.subsystem.swerve.SwerveDrive;
import frc.robot.Constants.*;
import frc.robot.commands.baseCommands.*;
import frc.robot.commands.complexCommands.*;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand.Position;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand.Output;
import frc.robot.commands.complexCommands.AutoPlaceCommand.Location;
import frc.robot.commands.complexCommands.PlaceCommand.GamePiece;
import frc.robot.commands.debugCommands.TiltIntakeCommand;


public class RobotContainer {
    /* Setting Joystick Buttons */
    private final CommandXboxController driveStick = new CommandXboxController(2);
    private final CommandJoystick controlJoystick = new CommandJoystick(1);

    private final Trigger switchDriveModeButton = driveStick.x();
    private final Trigger resetGyroButton = driveStick.a();
    private final Trigger slowModeButton = driveStick.leftBumper();
    private final Trigger driverPlaceButton = driveStick.b();

    private final Trigger cubeButton = controlJoystick.button(JoystickConstants.CUBE_INTAKE);
    private final Trigger placeButton = controlJoystick.button(JoystickConstants.PLACE);
    private final Trigger readySubstationButton = controlJoystick.button(JoystickConstants.SUBSTATION_PICKUP);
    private final Trigger readyTopButton = controlJoystick.button(JoystickConstants.READY_TOP);
    private final Trigger readyMidButton = controlJoystick.button(JoystickConstants.READY_MIDDLE);
    private final Trigger readyBotButton = controlJoystick.button(JoystickConstants.READY_BOTTOM);
    private final Trigger coneButton = controlJoystick.button(JoystickConstants.CONE_INTAKE);
    private final Trigger tiltUpButton = controlJoystick.button(4);
    private final Trigger tiltDownButton = controlJoystick.button(2);

    private final SwerveDrive m_swerve = new SwerveDrive();
    private final Arm m_arm = new Arm();
    private final Intake m_intake = new Intake();

    private SwerveDriveCommand swerveCommand;

    /** A hash map containing the commands the robot will use in auto <p> These commands can be accessed by putting the cooresponding string key into the .get() method
     * <p> Example: {@code autoCommandMap.get("zero");} 
     */
    public static final HashMap<String, Command> autoCommandMap = new HashMap<>();
    
    /**Both PID constants need to be tested */
    private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(m_swerve::getRobotPose, m_swerve::resetPose, new PIDConstants(5, 0, 0), new PIDConstants(4, 0, 0), m_swerve::driveModules, autoCommandMap, m_swerve);    
    private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

    public RobotContainer() {
        configureSwerve();
        configureAuto();
        configureArmAndIntake();
    }
    
    void configureSwerve() {
        swerveCommand = new SwerveDriveCommand(m_swerve, driveStick);
        m_swerve.setDefaultCommand(swerveCommand);

        switchDriveModeButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.switchControlMode();}));
        resetGyroButton.toggleOnTrue(new ResetGyroCommand(m_swerve));
        slowModeButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.slowSpeed();}));
        slowModeButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.fastSpeed();}));
        placeButton.and(() -> Intake.getGamePiece().get()== GamePiece.Cone).toggleOnTrue(
            new PlaceCommand(m_arm, m_intake, GamePiece.Cone)
        );

        placeButton.and(() -> Intake.getGamePiece().get() == GamePiece.Cube).toggleOnTrue(
            new PlaceCommand(m_arm, m_intake, GamePiece.Cube)
        );
        driverPlaceButton.toggleOnFalse(
            new ZeroCommand(m_arm, m_intake)
        );
        
        Shuffleboard.getTab("Swerve").add("Swerve", m_swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
    }



    void configureArmAndIntake() {

        tiltUpButton.toggleOnTrue(new TiltIntakeCommand(m_intake, 1));
        tiltDownButton.toggleOnTrue(new TiltIntakeCommand(m_intake, -1));

        cubeButton.toggleOnTrue( 
            new SetIntakeSpeedCommand(m_intake, Output.PickupCube)
        );
        cubeButton.toggleOnFalse(
            new ZeroCommand(m_arm, m_intake)
        );

        coneButton.toggleOnTrue(
            new SetIntakeSpeedCommand(m_intake, Output.PickupCone)
        );
        coneButton.toggleOnFalse(
            new ZeroCommand(m_arm, m_intake)
        );

        readyBotButton.toggleOnTrue(
            new SetArmAndIntakeCommand(m_arm, m_intake, Position.Low)
        );

        readyMidButton.toggleOnTrue(
            new SetArmAndIntakeCommand(m_arm, m_intake, Position.Middle)
        );

        readyTopButton.toggleOnTrue(
            new SetArmAndIntakeCommand(m_arm, m_intake, Position.High)
        );

        readySubstationButton.toggleOnTrue(
            new SetArmAndIntakeCommand(m_arm, m_intake, Position.Substation)
        );
       
        placeButton.and(() -> Intake.getGamePiece().get()== GamePiece.Cone).toggleOnTrue(
            new PlaceCommand(m_arm, m_intake, GamePiece.Cone)
        );

        placeButton.and(() -> Intake.getGamePiece().get() == GamePiece.Cube).toggleOnTrue(
            new PlaceCommand(m_arm, m_intake, GamePiece.Cube)
        );
        
        placeButton.toggleOnFalse(
            new ZeroCommand(m_arm, m_intake)
        );

        Shuffleboard.getTab("Arm and Intake").add("Intake", m_intake);
        Shuffleboard.getTab("Arm and Intake").add("Arm", m_arm);
        Shuffleboard.getTab("Arm and Intake").addString("Current Game Piece", () -> Intake.getGamePiece().get().name());
    }

    void configureAuto() {
        autoCommandMap.put(
            "start", 
            new SetArmAndIntakeCommand(m_arm, m_intake, Position.Start));

        autoCommandMap.put(
            "zero", 
            new ZeroCommand(m_arm, m_intake)
        );

        autoCommandMap.put(
            "placeCubeTop", 
            new AutoPlaceCommand(m_arm, m_intake, Location.HighCube)
        );

        autoCommandMap.put(
            "placeConeTop", 
            new AutoPlaceCommand(m_arm, m_intake, Location.HighCone)
        );

        autoCommandMap.put(
            "placeCubeMid", 
            new AutoPlaceCommand(m_arm, m_intake, Location.MidCube)
        );

        autoCommandMap.put(
            "placeConeMid", 
            new AutoPlaceCommand(m_arm, m_intake, Location.MidCone)
        );

        autoCommandMap.put(
            "resetGyro", 
            new ResetGyroCommand(m_swerve)
        );

        autoCommandMap.put(
            "reverseGyro", 
            new ResetGyroCommand(m_swerve, 180)
        );

        autoCommandMap.put(
            "pickupCone", 
            new AutoPickupCommand(m_arm, m_intake, GamePiece.Cone)
        );

        autoCommandMap.put(
            "pickupCube", 
            new AutoPickupCommand(m_arm, m_intake, GamePiece.Cube)
        );

        autonChooser.setDefaultOption("Blue Bottom: 2 Piece Top", autoBuilder.fullAuto(AutoConstants.BlueBotRedTop2PieceTopAuto));
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

}