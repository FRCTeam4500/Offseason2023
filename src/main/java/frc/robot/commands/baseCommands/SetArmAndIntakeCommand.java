package frc.robot.commands.baseCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Intake;
/**
 * A command that sets the Arm winch position, the Arm angle, and the Intake angle
 */
public class SetArmAndIntakeCommand extends CommandBase{
    private Arm arm;
    private Intake intake;
    private Position position;

    private double targetWinchPosition;
    private double targetArmAngle;
    private double targetIntakeAngle;

    /**
     * The constructor of SetArmAndIntakeCommand
     * @param arm the arm subsystem
     * @param intake the intake subsystem
     * @param position the desired position
     */
    public SetArmAndIntakeCommand(Arm arm, Intake intake, Position position) {
        this.arm = arm;
        this.intake = intake;
        this.position = position;
    }

    /**
     * The set of position groups that might be used
     */
    public enum Position {
        /** The position to pickup from the high substation */
        Substation,
        /** The position to place pieces on the high level */
        High,
        /** The position to place pieces on the middle level */
        Middle,
        /** The position to pickup from the ground */
        Low,
        /** The position to travel */
        Zero,
        /** The position to unlatch the arm when the match starts */
        Start
    }

    public void initialize() {
        switch (position) {
            case Substation:
                targetWinchPosition = ArmConstants.ARM_PLACE_TOP;
                targetArmAngle = ArmConstants.ARM_HIGH_SUBSTATION_ANGLE;
                targetIntakeAngle = IntakeConstants.INTAKE_HIGH_SUBSTATION_ANGLE;
                break;
            case High:
                targetWinchPosition = ArmConstants.ARM_PLACE_TOP;
                targetArmAngle = ArmConstants.ARM_LAUNCH_ANGLE;
                targetIntakeAngle = IntakeConstants.INTAKE_LAUNCHING_ANGLE;
                break;
            case Middle:
                targetWinchPosition = ArmConstants.ARM_PLACE_MID;
                targetArmAngle = ArmConstants.ARM_PLACE_ANGLE;
                targetIntakeAngle = IntakeConstants.INTAKE_TOP_CONE_PLACE_ANGLE;
                break;
            case Low:
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
