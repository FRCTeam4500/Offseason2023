package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.EnumConstants.IntakeSpeed;
import frc.robot.Constants.EnumConstants.PlacerState;
import frc.robot.commands.baseCommands.RumbleCommand;
import frc.robot.commands.baseCommands.SetArmAndIntakeCommand;
import frc.robot.commands.baseCommands.SetIntakeSpeedCommand;
import frc.robot.commands.complexCommands.PlaceCommand;
import frc.robot.commands.complexCommands.ZeroCommand;
import frc.robot.commands.debugCommands.TiltIntakeCommand;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;

public class OperatorController extends CommandJoystick {

    Intake intake;
    Arm arm;

    private DriveController driveController = DriveController.getInstance();

    private static OperatorController instanceOperatorController = null;

    private final Trigger cubeButton = this.button(JoystickConstants.CUBE_INTAKE);
    private final Trigger placeButton = this.button(JoystickConstants.PLACE);
    private final Trigger readySubstationButton = this.button(JoystickConstants.SUBSTATION_PICKUP);
    private final Trigger readyTopButton = this.button(JoystickConstants.READY_TOP);
    private final Trigger readyMidButton = this.button(JoystickConstants.READY_MIDDLE);
    private final Trigger readyBotButton = this.button(JoystickConstants.READY_BOTTOM);
    private final Trigger uprightConeButton = this.button(JoystickConstants.UPRIGHT_CONE_INTAKE);
    private final Trigger tiltedConeButton = this.button(JoystickConstants.TILTED_CONE_INTAKE);
    private final Trigger tiltUpButton = this.button(4);
    private final Trigger tiltDownButton = this.button(2);
    
    private OperatorController(int port, Intake intake, Arm arm) {
        super(port);
        this.intake = intake;
        this.arm = arm;

        setPlacerButtons();
    }

    /**
     * Creates a new instance of the OperatorController. If the instance is null, it will create a new one.
     * Should be called
     * @param port
     * @param intake
     * @param arm
     * @return
     */
    public static synchronized OperatorController getInstance(int port, Intake intake, Arm arm) {
        if (instanceOperatorController == null) {
            instanceOperatorController = new OperatorController(port, intake, arm);
        }
        return instanceOperatorController;
    }

    /**
     * Returns the instance of the OperatorController. If the instance is null, it will return null.
     * @return
     */
    public static synchronized OperatorController getInstance() {
        return instanceOperatorController;
    }

    public void setPlacerButtons() {
        tiltUpButton.toggleOnTrue(new TiltIntakeCommand(intake, 1));
        tiltDownButton.toggleOnTrue(new TiltIntakeCommand(intake, -1));

        cubeButton.toggleOnTrue( 
            new SetIntakeSpeedCommand(intake, IntakeSpeed.PickupCube)
        );
        cubeButton.toggleOnFalse(
            new ZeroCommand(arm, intake)
        );

        uprightConeButton.toggleOnTrue(
            new SetIntakeSpeedCommand(intake, IntakeSpeed.PickupUprightCone)
        );
        uprightConeButton.toggleOnFalse(
            new ZeroCommand(arm, intake)
        );

        tiltedConeButton.toggleOnTrue(
            new SetIntakeSpeedCommand(intake, IntakeSpeed.PickupTiltedCone)
        );
        tiltedConeButton.toggleOnFalse(
            new ZeroCommand(arm, intake)
        );

        readyBotButton.toggleOnTrue(
            new SetArmAndIntakeCommand(arm, intake, PlacerState.GroundPickup)
        );

        readyMidButton.and(() -> Intake.getGamePiece().get() == GamePiece.Cube).toggleOnTrue(
            new SetArmAndIntakeCommand(arm, intake, PlacerState.MidCube)
        );

        readyMidButton.and(() -> Intake.getGamePiece().get() == GamePiece.UprightCone).toggleOnTrue(
          new SetArmAndIntakeCommand(arm, intake, PlacerState.MidUprightCone)  
        );

        readyMidButton.and(() -> Intake.getGamePiece().get() == GamePiece.TiltedCone).toggleOnTrue(
          new SetArmAndIntakeCommand(arm, intake, PlacerState.MidTiltedCone)  
        );

        readyTopButton.and(() -> Intake.getGamePiece().get() == GamePiece.Cube).toggleOnTrue(
            new SetArmAndIntakeCommand(arm, intake, PlacerState.HighCube)
        );

        readyTopButton.and(() -> Intake.getGamePiece().get() == GamePiece.UprightCone).toggleOnTrue(
            new SetArmAndIntakeCommand(arm, intake, PlacerState.HighUprightCone)
        );

        readyTopButton.and(() -> Intake.getGamePiece().get() == GamePiece.TiltedCone).toggleOnTrue(
            new RumbleCommand(driveController, .5)
        );

        readySubstationButton.toggleOnTrue(
            new SetArmAndIntakeCommand(arm, intake, PlacerState.SubstationPickup)
        );
       
        placeButton.and(() -> Intake.getGamePiece().get() == GamePiece.TiltedCone || 
        Intake.getGamePiece().get() == GamePiece.UprightCone).toggleOnTrue(
            new PlaceCommand(arm, intake, GamePiece.UprightCone)
        );

        placeButton.and(() -> Intake.getGamePiece().get() == GamePiece.Cube).toggleOnTrue(
            new PlaceCommand(arm, intake, GamePiece.Cube)
        );
        
        placeButton.toggleOnFalse(
            new ZeroCommand(arm, intake)
        );
    }

}
