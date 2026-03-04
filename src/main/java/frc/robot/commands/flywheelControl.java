package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.flywheel;

public class flywheelControl extends Command {
	private final flywheel m_flywheel;
	private DoubleSupplier triggerAxis;

	private double lowerSpeed = .5;
	private double RPM = 500;

	private double firstThreshold = 1/3.;
	private double secondThreshold = 2/3.;

	public flywheelControl(flywheel flywheel, CommandXboxController x) {
		triggerAxis = x::getLeftTriggerAxis;
		m_flywheel = flywheel;
		addRequirements(m_flywheel);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		m_flywheel.setLower(lowerSpeed * ((triggerAxis.getAsDouble() > secondThreshold) ? 1:0));
		m_flywheel.setTargetRPM(RPM * ((triggerAxis.getAsDouble() > firstThreshold) ? 1:0));
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
