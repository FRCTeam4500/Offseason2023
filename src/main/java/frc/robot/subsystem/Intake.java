package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.component.hardware.SparkMaxComponent;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.util.sendable.SendableBuilder;


public class Intake extends SubsystemBase {
    private SparkMaxComponent intakeMotor;
    public SparkMaxComponent intakeTiltMotor;
    private double targetTiltAngle;
    private double targetIntakeOutput;
    private SparkMaxPIDController tiltPIDController;
    private static boolean isCone2;

    public Intake() {
        this.intakeMotor = new SparkMaxComponent(IntakeConstants.INTAKE_MOTOR_ID, IntakeConstants.INTAKE_MOTOR_TYPE);
        this.intakeTiltMotor = new SparkMaxComponent(IntakeConstants.INTAKE_ANGLE_MOTOR_ID, IntakeConstants.ANGLE_MOTOR_TYPE);

        this.intakeMotor.setIdleMode(IdleMode.kBrake);

        this.intakeTiltMotor.setIdleMode(IdleMode.kCoast);

        this.tiltPIDController = this.intakeTiltMotor.getPIDController();
        this.tiltPIDController.setP(1);
        this.tiltPIDController.setI(0);
        this.tiltPIDController.setD(0);

        this.tiltPIDController.setOutputRange(-.3, .3);
        this.intakeTiltMotor.setSoftLimit(SoftLimitDirection.kReverse, -40f);
    }

    public Intake(SparkMaxComponent intakeMotor, SparkMaxComponent intakeAngleMotor) {
        this.intakeMotor = intakeMotor;
        this.intakeTiltMotor = intakeAngleMotor;
    }

    public void setSpeed(double speed) {
        targetIntakeOutput = speed;
        intakeMotor.set(speed);
    }

    public double getSpeed() {
        return intakeMotor.get();
    }
    public void setAngle(double angle) {
        targetTiltAngle = angle;
        intakeTiltMotor.setAngle(angle);
    }

    public double getAngle() {
        return intakeTiltMotor.getEncoder().getPosition();
    }

    public void setAngleOutput(double output) {
        intakeTiltMotor.setOutput(output);
    }

    public static class IntakeSetAngleCommand extends InstantCommand {
        private Intake intake;
        private double angle;

        

        public IntakeSetAngleCommand(Intake intake, double angle){
            this.intake = intake;
            this.angle = angle;
        }

        public IntakeSetAngleCommand(Intake intake){
            this.intake = intake;
            this.angle = IntakeConstants.INTAKE_TOP_CONE_PLACE_ANGLE;
        }

        
        

        @Override
        public void initialize() {
            intake.setAngle(angle);
        }
    }

    public static class IntakeSetOutputCommand extends InstantCommand {
        private Intake intake;
        private double speed;


        public IntakeSetOutputCommand(Intake intake, double speed){
            this.intake = intake;
            this.speed = speed;
        }

        @Override
        public void initialize() {
            intake.setSpeed(speed);
        }
    }

    public static class IntakeSetAngleOutputCommand extends CommandBase {
        Intake intake;
        double output;
        
        public IntakeSetAngleOutputCommand(Intake intake, double output) {
            this.intake = intake;
            this.output = output;
        }

        @Override
        public void initialize() {
            intake.setAngleOutput(output);
        }

    }

    public static class IntakeChangeTiltCommand extends InstantCommand {
        private Intake intake;
        private double addition;

        public IntakeChangeTiltCommand(Intake intake, double addition){
            this.intake = intake;
            this.addition = addition;
        }

        public void initialize(){
            intake.setAngle(intake.intakeTiltMotor.getEncoder().getPosition() + addition);
        }
    }

    public static Intake makeIntake() {
        return new Intake();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target Intake Tilt position", () -> targetTiltAngle, (value) -> {setAngle((double) value);});
        builder.addDoubleProperty("Target Intake Wheel output", () -> targetIntakeOutput, (value) -> {setSpeed((double) value);});
        builder.addDoubleProperty("Intake Angle", () -> intakeTiltMotor.getEncoder().getPosition(), null);
    }
}
