package frc.robot.subsystem.placer;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem.placer.arm.Arm;
import frc.robot.subsystem.placer.intake.Intake;

public class Placer extends SubsystemBase {

	double WIDTH = 4.0;
	double HEIGHT = 4.0;

	double pivotHeight = 2.0;
	double minimumExtention = 0.762;
	double armZeroAngle = 1.138;
	double intakeLength = 0.487;

	Mechanism2d mechanism2d;

	MechanismRoot2d armPivot;
	MechanismLigament2d armExtention;
	MechanismLigament2d armRest;
	MechanismLigament2d intakeAngle;

	private static Placer instancePlacer;

	private Placer() {
		mechanism2d = new Mechanism2d(WIDTH, HEIGHT);
		armPivot = mechanism2d.getRoot("Arm Pivot", WIDTH / 2, pivotHeight);
		armExtention =
			armPivot.append(
				new MechanismLigament2d(
					"extention",
					minimumExtention,
					armZeroAngle
				)
			);
		armRest =
			armPivot.append(
				new MechanismLigament2d(
					"Arm Rest",
					minimumExtention,
					armZeroAngle + 180
				)
			);
		intakeAngle =
			armExtention.append(
				new MechanismLigament2d("intake", intakeLength, 90)
			);
		Shuffleboard.getTab("placer").add("Placer", mechanism2d);
	}

	public static synchronized Placer getInstance() {
		if (instancePlacer == null) {
			instancePlacer = new Placer();
		}
		return instancePlacer;
	}

	public Mechanism2d getMechanism2d() {
		return mechanism2d;
	}

	public void setArmAngle(double angle) {
		armExtention.setAngle(angle);
		armRest.setAngle(angle + 180);
	}

	@Override
	public void periodic() {
		setArmAngle(-Math.toDegrees(Arm.getInstance().getAngle()));
		armExtention.setLength(
			minimumExtention + Arm.getInstance().getExtension()
		);
		intakeAngle.setAngle(Math.toDegrees(Intake.getInstance().getAngle()));
	}
}
