package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.component.SparkMaxComponent;

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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target Intake Tilt position", () -> targetTiltAngle, (value) -> {setAngle((double) value);});
        builder.addDoubleProperty("Target Intake Wheel output", () -> targetIntakeOutput, (value) -> {setSpeed((double) value);});
        builder.addDoubleProperty("Intake Angle", () -> intakeTiltMotor.getEncoder().getPosition(), null);
    }
}
