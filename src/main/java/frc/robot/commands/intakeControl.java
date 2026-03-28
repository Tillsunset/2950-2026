package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake;

public class intakeControl extends Command {
	private final intake m_intake;
	private DoubleSupplier triggerAxis;
	

	// 1 unit = 1 rotation
	// encoder is reset to 0 on robot start, and teleop start
	private double armMin = (15/360.) * (32/12.) * (45/1.);
	// 110 degrees of arm rotaion, 32:12 reduction, 45:1 gear reduction
	private double armMax = (110/360.) * (32/12.) * (45/1.);

	public intakeControl(intake intake, CommandXboxController x) {
		triggerAxis = x::getRightTriggerAxis;
		m_intake = intake;
		addRequirements(m_intake);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		// scales the first half of the left trigger to arm position
		double armScaled = armMin + (armMax - armMin) * (Math.min(0.5, Math.abs(triggerAxis.getAsDouble())) / 0.5);
				// scales the first half trigger to full wheel
				double wheelScaled = (Math.min(0.5, Math.abs(triggerAxis.getAsDouble())) / 0.5);

		m_intake.setWheel(wheelScaled);
		m_intake.updateTargetAngle(armScaled);

	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
