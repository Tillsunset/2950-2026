package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake;

public class intakeControl extends Command {
	private final intake m_intake;
	private DoubleSupplier triggerAxis;

	private double armMin = 0;
	private double armMax = 32 * 45/(4.0 * 12);

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
		// scales the first half of the left trigger to arm position and the second half to wheel speed
		double armScaled = armMin + (armMax - armMin) * Math.min(1, Math.abs(triggerAxis.getAsDouble() * 2));
		double wheelScaled = Math.abs(triggerAxis.getAsDouble());

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
