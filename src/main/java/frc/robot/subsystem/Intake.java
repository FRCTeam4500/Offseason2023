package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.component.SparkMaxComponent;

import java.util.function.DoubleSupplier;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.util.sendable.SendableBuilder;


public class Intake extends SubsystemBase {
    private SparkMaxComponent speedMotor;
    public SparkMaxComponent angleMotor;
    private SparkMaxPIDController anglePIDController;
    private double targetAngle;
    private double targetSpeed;

    public Intake() {
        speedMotor = new SparkMaxComponent(IntakeConstants.INTAKE_MOTOR_ID, IntakeConstants.INTAKE_MOTOR_TYPE);
        angleMotor = new SparkMaxComponent(IntakeConstants.INTAKE_ANGLE_MOTOR_ID, IntakeConstants.ANGLE_MOTOR_TYPE);

        speedMotor.setIdleMode(IdleMode.kBrake);

        angleMotor.setIdleMode(IdleMode.kCoast);
        angleMotor.setSoftLimit(SoftLimitDirection.kReverse, -40);

        anglePIDController = angleMotor.getPIDController();
        anglePIDController.setP(1);
        anglePIDController.setI(0);
        anglePIDController.setD(0);
        anglePIDController.setOutputRange(-.3, .3);
    }

    public void setSpeed(double speed) {
        targetSpeed = speed;
        speedMotor.set(speed);
    }

    public DoubleSupplier getSpeed() {
        return () -> speedMotor.getOutput();
    }

    public DoubleSupplier getTargetSpeed() {
        return () -> targetSpeed;
    }

    public void setAngle(double angle) {
        targetAngle = angle;
        angleMotor.setAngle(angle);
    }

    public DoubleSupplier getAngle() {
        return () -> angleMotor.getAngle();
    }

    public DoubleSupplier getTargetAngle() {
        return () -> targetAngle;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target Intake Tilt position", getTargetAngle(), null);
        builder.addDoubleProperty("Target Intake Wheel output", getTargetSpeed(), null);
        builder.addDoubleProperty("Intake Angle", getAngle(), null);
    }
}
