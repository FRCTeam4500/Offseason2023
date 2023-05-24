package frc.robot.subsystem.placer;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Placer extends SubsystemBase {

	double WIDTH = 3.0;
	double HEIGHT = 3.0;

	Mechanism2d mechanism2d;

	MechanismRoot2d armPivot;
	MechanismLigament2d armExtention;
	MechanismLigament2d intakeAngle;

	public Placer() {
		mechanism2d = new Mechanism2d(WIDTH, HEIGHT); //TODO: add real values
		armPivot = mechanism2d.getRoot("Arm Pivot", WIDTH / 2, 0);
		armExtention =
			armPivot.append(new MechanismLigament2d("extention", .5, 0));
		intakeAngle =
			armExtention.append(new MechanismLigament2d("intake", .3, 90));
		Shuffleboard.getTab("placer").add("Placer", mechanism2d);
	}

	@Override
	public void periodic() {
		armExtention.setAngle(armExtention.getAngle() + 1);
		intakeAngle.setAngle(intakeAngle.getAngle() + 1);
	}
}
