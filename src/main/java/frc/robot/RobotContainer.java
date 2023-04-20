package frc.robot;

import static frc.robot.subsystem.Arm.makeArm;
import static frc.robot.subsystem.Intake.makeIntake;

import static frc.robot.Constants.RobotConstants.commandMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.autoCommands.AutoPickupCommand;
import frc.robot.commands.autoCommands.AutoPlaceCommand;
import frc.robot.commands.autoCommands.AutoPickupCommand.GamePiece;
import frc.robot.commands.autoCommands.AutoPlaceCommand.PlacePosition;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand.Position;
import frc.robot.commands.teleOpCommands.SetConsecutiveIntakeOutputs;
import frc.robot.commands.teleOpCommands.SwerveDriveCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.HardwareSwerveFactory;
import frc.robot.subsystem.swerve.pathfollowingswerve.OdometricSwerve;


public class RobotContainer {
    /* Setting Joystick Buttons */
    private final CommandXboxController driveStick = new CommandXboxController(2);
    private final Joystick controlStick = new Joystick(1);

    private final Trigger switchDriveModeButton = driveStick.x();
    private final Trigger resetGyroButton = driveStick.a();
    private final Trigger slowModeButton = driveStick.leftBumper();
    private final Trigger driverPlaceButton = driveStick.b();

    private final JoystickButton cubeButton = new JoystickButton(controlStick, JoystickConstants.CUBE_INTAKE);
    private final JoystickButton placeButton = new JoystickButton(controlStick, JoystickConstants.PLACE);
    private final JoystickButton readySubstationButton = new JoystickButton(controlStick, JoystickConstants.SUBSTATION_PICKUP);
    private final JoystickButton readyTopButton = new JoystickButton(controlStick, JoystickConstants.READY_TOP);
    private final JoystickButton readyMidButton = new JoystickButton(controlStick, JoystickConstants.READY_MIDDLE);
    private final JoystickButton readyBotButton = new JoystickButton(controlStick, JoystickConstants.READY_BOTTOM);
    private final JoystickButton uprightConeButton = new JoystickButton(controlStick, JoystickConstants.UPRIGHT_CONE);
    
    private final JoystickButton tiltUp = new JoystickButton(controlStick, 4);
    private final JoystickButton tiltDown = new JoystickButton(controlStick, 2);
    private SwerveDriveCommand swerveCommand;

    private final OdometricSwerve m_swerve = HardwareSwerveFactory.makeSwerve();
    private final Arm m_arm = makeArm();
    private final Intake m_intake = makeIntake();

    /**Both PID constants need to be tested */
    private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(m_swerve::getCurrentPose, m_swerve::resetPose, new PIDConstants(5, 0, 0), new PIDConstants(4, 0, 0), m_swerve::moveRobotCentric, commandMap, m_swerve);    
    private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

    public RobotContainer() {
        configureCommands();
        configureSwerve();
        configureAuto();
        configureArmAndIntake();
    }
    
    void configureSwerve() {
        swerveCommand = new SwerveDriveCommand(m_swerve, driveStick);
        m_swerve.setDefaultCommand(swerveCommand);

        switchDriveModeButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.switchControlMode();}));
        resetGyroButton.toggleOnTrue(new InstantCommand(() -> {m_swerve.resetRobotAngle();}));
        slowModeButton.toggleOnTrue(new InstantCommand(() -> {swerveCommand.slowSpeed();}));
        slowModeButton.toggleOnFalse(new InstantCommand(() -> {swerveCommand.fastSpeed();}));
        driverPlaceButton.toggleOnTrue(
            commandMap.get("place")
        );
        driverPlaceButton.toggleOnFalse(
            commandMap.get("zero")
        );
        
        Shuffleboard.getTab("Swerve").add("Swerve", m_swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);
    }


    void configureCommands() {
        commandMap.put(
            "readyBot", 
            new SetArmAndIntakeCommand(m_arm, m_intake, Position.Low)
        );

        commandMap.put(
            "readyMid",
            new SetArmAndIntakeCommand(m_arm, m_intake, Position.Middle)
        );

        commandMap.put(
            "readyTop",
            new SetArmAndIntakeCommand(m_arm, m_intake, Position.High)
        );

        commandMap.put(
            "readySubstation",
            new SetArmAndIntakeCommand(m_arm, m_intake, Position.Substation)
        );

        commandMap.put(
            "place",
            new SetConsecutiveIntakeOutputs(m_intake, IntakeConstants.INTAKE_CONE_SPEED, 0.5, IntakeConstants.INTAKE_CUBE_SPEED)
        );

        commandMap.put(
            "intakeCube",
            new SetIntakeSpeedCommand(m_intake, IntakeConstants.INTAKE_CUBE_SPEED)
        );

        commandMap.put(
            "intakeCone", 
            new SetIntakeSpeedCommand(m_intake, IntakeConstants.INTAKE_CONE_SPEED)  
        );
        
        commandMap.put(
            "zero",
            new SequentialCommandGroup(
                new SetArmAndIntakeCommand(m_arm, m_intake, Position.Zero),
                new SetIntakeSpeedCommand(m_intake, 0)
            )
        );

        commandMap.put(
            "start",
            new SetArmAndIntakeCommand(m_arm, m_intake, Position.Start)
        );

        commandMap.put(
            "waitQuarter", 
            new WaitCommand(.25)
        );

        commandMap.put(
            "waitHalf", 
            new WaitCommand(.5)
        );
        
        commandMap.put(
            "waitOne", 
            new WaitCommand(1)
        );

        commandMap.put(
            "Reset Gyro", 
            new InstantCommand(() -> {m_swerve.resetRobotAngle();})
        );

        commandMap.put(
            "Reverse Gyro", 
            new InstantCommand(() -> {m_swerve.resetRobotAngle(180);})
        );

        commandMap.put(
            "autoPlaceConeTop",
            new AutoPlaceCommand(m_arm, m_intake, PlacePosition.HighCone)
        );

        commandMap.put(
            "autoPlaceConeMid",
            new AutoPlaceCommand(m_arm, m_intake, PlacePosition.MiddleCone)
        );

        commandMap.put(
            "autoPlaceCubeTop",
            new AutoPlaceCommand(m_arm, m_intake, PlacePosition.HighCube)
        );

        commandMap.put(
            "autoPlaceCubeMid",
            new AutoPlaceCommand(m_arm, m_intake, PlacePosition.MiddleCube)
        );

        commandMap.put(
            "autoPickupCube",
            new AutoPickupCommand(m_arm, m_intake, GamePiece.Cube)
        );

        commandMap.put(
            "autoPickupCone",
            new AutoPickupCommand(m_arm, m_intake, GamePiece.Cone)
        );
    }

    void configureArmAndIntake() {

        tiltUp.toggleOnTrue(new Intake.IntakeChangeTiltCommand(m_intake, 1));
        tiltDown.toggleOnTrue(new Arm.ArmChangeTiltCommand(m_arm, -1));
        cubeButton.toggleOnTrue( 
            commandMap.get("intakeCube")
        );
        cubeButton.toggleOnFalse(
            commandMap.get("zero")
        );

        uprightConeButton.toggleOnTrue(
            commandMap.get("intakeCone")
        );
        uprightConeButton.toggleOnFalse(
            commandMap.get("zero") 
        );

        readyBotButton.toggleOnTrue(
            commandMap.get("readyBot")
        );

        readyMidButton.toggleOnTrue(
            commandMap.get("readyMid")
        );

        readyTopButton.toggleOnTrue(
            commandMap.get("readyTop")
        );

        readySubstationButton.toggleOnTrue(
            commandMap.get("readySubstation")
        );
       
        placeButton.toggleOnTrue(
            commandMap.get("place")
        );
        placeButton.toggleOnFalse(
            commandMap.get("zero")
        );

        Shuffleboard.getTab("Arm and Intake").add("Intake", m_intake);
        Shuffleboard.getTab("Arm and Intake").add("Arm", m_arm);
    }

    void configureAuto() {
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