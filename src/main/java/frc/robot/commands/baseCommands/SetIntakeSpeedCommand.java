package frc.robot.commands.baseCommands;

import java.util.Map;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.EnumConstants.GamePiece;
import frc.robot.Constants.EnumConstants.IntakeSpeed;
import frc.robot.subsystem.Intake;

public class SetIntakeSpeedCommand extends CommandBase{
    private Intake intake;
    private IntakeSpeed targetSpeed;
    private double rawTargetSpeed;
    private ComplexWidget thisWidget;
    private static int timesRun = 0;
    public SetIntakeSpeedCommand(Intake intake, IntakeSpeed speed) {
        this.intake = intake;
        this.targetSpeed = speed;
    }

    @Override
    public void initialize() {
        timesRun++;
        thisWidget = Shuffleboard.getTab("Commands").getLayout("Command Grid", BuiltInLayouts.kList).add("Intake Speed Command #" + timesRun, this);
        switch (targetSpeed) {
            case PickupCube:
                Intake.setGamePiece(GamePiece.Cube);
            case PlaceCone: 
                Intake.setGamePiece(GamePiece.Nothing);
                rawTargetSpeed = IntakeConstants.OUTTAKE_CONE_SPEED;
                break;
            case PickupUprightCone:
                Intake.setGamePiece(GamePiece.UprightCone);
                rawTargetSpeed = IntakeConstants.INTAKE_CONE_SPEED;
                break;
            case PickupTiltedCone:
                Intake.setGamePiece(GamePiece.TiltedCone);
                rawTargetSpeed = IntakeConstants.INTAKE_CONE_SPEED;
                break;
            case PlaceCube: 
                Intake.setGamePiece(GamePiece.Nothing);
                rawTargetSpeed = IntakeConstants.OUTTAKE_CUBE_SPEED;
                break;
            case Off:
                rawTargetSpeed = 0;
                break;
        }
        intake.setSpeed(rawTargetSpeed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rawTargetSpeed - intake.getSpeed().getAsDouble()) < IntakeConstants.INTAKE_SPEED_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        thisWidget.withProperties(Map.of("Label position", "HIDDEN"));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target Intake Speed: ", () -> rawTargetSpeed, null);
        builder.addDoubleProperty("Difference to Target Speed: ", () -> Math.abs(rawTargetSpeed - intake.getSpeed().getAsDouble()), null);
    }
}
