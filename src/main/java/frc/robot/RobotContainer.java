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

    private final SwerveDrive swerve = SwerveDrive.getInstance();
    private final Arm arm = Arm.getInstance();
    private final Intake intake = Intake.getInstance();

    /* Setting controller Buttons */
    private final DriveController driveStick = DriveController.getInstance(JoystickConstants.DRIVER_PORT, intake, arm, swerve);
    private final OperatorController controlJoystick = OperatorController.getInstance(JoystickConstants.OPERATOR_PORT, intake, arm);

    private final Autonomous autonomous = Autonomous.getInstance(swerve, arm, intake);

    public RobotContainer() { }

    public Command getAutonomousCommand() {
        return autonomous.getAutonCommand();
    }

    public void teleopInit() {
        Command auton = autonomous.getAutonCommand();
        if (auton != null){
            auton.cancel();
        }
    }

    public void disabledInit() {
        swerve.zeroModules();
    }
}