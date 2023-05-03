package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.EnumConstants.PlacerState;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;
/**
 * A command that sets the Arm winch state, the Arm angle, and the Intake angle
 */
public class SetArmAndIntakeCommand extends CommandBase{
    private Arm arm;
    private Intake intake;
    private PlacerState state;

    private double targetWinchPosition;
    private double targetArmAngle;
    private double targetIntakeAngle;

    /**
     * The constructor of SetArmAndIntakeCommand
     * @param arm the arm subsystem
     * @param intake the intake subsystem
     * @param state the desired state
     */
    public SetArmAndIntakeCommand(Arm arm, Intake intake, PlacerState state) {
        this.arm = arm;
        this.intake = intake;
        this.state = state;
    }


    public void initialize() {
        switch (state) {
            case SubstationPickup:
                targetWinchPosition = ArmConstants.ARM_PLACE_TOP;
                targetArmAngle = ArmConstants.ARM_HIGH_SUBSTATION_ANGLE;
                targetIntakeAngle = IntakeConstants.INTAKE_HIGH_SUBSTATION_ANGLE;
                break;
            case HighUprightCone: case HighCube:
                targetWinchPosition = ArmConstants.ARM_PLACE_TOP;
                targetArmAngle = ArmConstants.ARM_LAUNCH_ANGLE;
                targetIntakeAngle = IntakeConstants.INTAKE_LAUNCHING_ANGLE;
                break;
            case MidCube: case MidUprightCone:
                targetWinchPosition = ArmConstants.ARM_PLACE_MID;
                targetArmAngle = ArmConstants.ARM_PLACE_ANGLE;
                targetIntakeAngle = IntakeConstants.INTAKE_TOP_CONE_PLACE_ANGLE;
                break;
            case MidTiltedCone:
                targetWinchPosition = ArmConstants.ARM_PLACE_TILTED_CONE_MID;
                targetArmAngle = ArmConstants.ARM_PLACE_TILTED_CONE_ANGLE;
                targetIntakeAngle = IntakeConstants.INTAKE_TILTED_CONE_ANGLE;
                break;
            case GroundPickup:
                targetWinchPosition = ArmConstants.ARM_PICKUP;
                targetArmAngle = ArmConstants.ARM_GROUND_ANGLE;
                targetIntakeAngle = IntakeConstants.INTAKE_BOT_ANGLE;
                break;
            case Zero:
                targetWinchPosition = ArmConstants.ARM_RETRACT;
                targetArmAngle = ArmConstants.ARM_ZERO_ANGLE;
                targetIntakeAngle = IntakeConstants.INTAKE_ZERO_ANGLE;
                break;
            case Start:
                targetWinchPosition = ArmConstants.ARM_RETRACT;
                targetArmAngle = 0;
                targetIntakeAngle = IntakeConstants.INTAKE_ZERO_ANGLE;
                intake.setSpeed(0);
                break;
        }
        
        arm.setWinchPosition(targetWinchPosition);
        arm.setTilt(targetArmAngle);
        intake.setAngle(targetIntakeAngle);
    }

    public boolean isFinished() {
        return true;
    }
}
