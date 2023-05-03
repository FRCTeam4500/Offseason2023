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
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.EnumConstants.IntakeSpeed;
import frc.robot.Constants.EnumConstants.PlacerState;
import frc.robot.commands.baseCommands.*;
import frc.robot.commands.complexCommands.*;
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
    private final Trigger uprightConeButton = controlJoystick.button(JoystickConstants.UPRIGHT_CONE_INTAKE);
    private final Trigger tiltedConeButton = controlJoystick.button(JoystickConstants.TILTED_CONE_INTAKE);
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

        driverPlaceButton.and(() -> Intake.getGamePiece().get() == GamePiece.TiltedCone || 
        Intake.getGamePiece().get() == GamePiece.UprightCone).toggleOnTrue(
            new PlaceCommand(m_arm, m_intake, GamePiece.UprightCone)
        );

        driverPlaceButton.and(() -> Intake.getGamePiece().get() == GamePiece.Cube).toggleOnTrue(
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
            new SetIntakeSpeedCommand(m_intake, IntakeSpeed.PickupCube)
        );
        cubeButton.toggleOnFalse(
            new ZeroCommand(m_arm, m_intake)
        );

        uprightConeButton.toggleOnTrue(
            new SetIntakeSpeedCommand(m_intake, IntakeSpeed.PickupUprightCone)
        );
        uprightConeButton.toggleOnFalse(
            new ZeroCommand(m_arm, m_intake)
        );

        tiltedConeButton.toggleOnTrue(
            new SetIntakeSpeedCommand(m_intake, IntakeSpeed.PickupTiltedCone)
        );
        tiltedConeButton.toggleOnFalse(
            new ZeroCommand(m_arm, m_intake)
        );

        readyBotButton.toggleOnTrue(
            new SetArmAndIntakeCommand(m_arm, m_intake, PlacerState.GroundPickup)
        );

        readyMidButton.and(() -> Intake.getGamePiece().get() == GamePiece.Cube).toggleOnTrue(
            new SetArmAndIntakeCommand(m_arm, m_intake, PlacerState.MidCube)
        );

        readyMidButton.and(() -> Intake.getGamePiece().get() == GamePiece.UprightCone).toggleOnTrue(
          new SetArmAndIntakeCommand(m_arm, m_intake, PlacerState.MidUprightCone)  
        );

        readyMidButton.and(() -> Intake.getGamePiece().get() == GamePiece.TiltedCone).toggleOnTrue(
          new SetArmAndIntakeCommand(m_arm, m_intake, PlacerState.MidTiltedCone)  
        );

        readyTopButton.and(() -> Intake.getGamePiece().get() == GamePiece.Cube).toggleOnTrue(
            new SetArmAndIntakeCommand(m_arm, m_intake, PlacerState.HighCube)
        );

        readyTopButton.and(() -> Intake.getGamePiece().get() == GamePiece.UprightCone).toggleOnTrue(
            new SetArmAndIntakeCommand(m_arm, m_intake, PlacerState.HighUprightCone)
        );

        readyTopButton.and(() -> Intake.getGamePiece().get() == GamePiece.TiltedCone).toggleOnTrue(
            new RumbleCommand(driveStick, .5)
        );

        readySubstationButton.toggleOnTrue(
            new SetArmAndIntakeCommand(m_arm, m_intake, PlacerState.SubstationPickup)
        );
       
        placeButton.and(() -> Intake.getGamePiece().get() == GamePiece.TiltedCone || 
        Intake.getGamePiece().get() == GamePiece.UprightCone).toggleOnTrue(
            new PlaceCommand(m_arm, m_intake, GamePiece.UprightCone)
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
            new SetArmAndIntakeCommand(m_arm, m_intake, PlacerState.Start));

        autoCommandMap.put(
            "zero", 
            new ZeroCommand(m_arm, m_intake)
        );

        autoCommandMap.put(
            "placeCubeTop", 
            new AutoPlaceCommand(m_arm, m_intake, PlacerState.HighCube)
        );

        autoCommandMap.put(
            "placeConeTop", 
            new AutoPlaceCommand(m_arm, m_intake, PlacerState.HighUprightCone)
        );

        autoCommandMap.put(
            "placeCubeMid", 
            new AutoPlaceCommand(m_arm, m_intake, PlacerState.MidCube)
        );

        autoCommandMap.put(
            "placeConeMid", 
            new AutoPlaceCommand(m_arm, m_intake, PlacerState.MidUprightCone)
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
            new AutoPickupCommand(m_arm, m_intake, GamePiece.UprightCone)
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