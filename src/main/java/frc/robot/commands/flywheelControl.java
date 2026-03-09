package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.flywheel;

public class flywheelControl extends Command {
	private final flywheel m_flywheel;
	private DoubleSupplier triggerAxis;

	private double feederPercent = .5;
	private double flywheelRPM = 1000;

	private double firstThreshold = 0.3;
	private double secondThreshold = 0.7;

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
		// on/off control
		m_flywheel.setTargetRPM(flywheelRPM * 	((Math.abs(triggerAxis.getAsDouble()) > firstThreshold)  ? 1 : 0)); // first start flywheel 
		m_flywheel.setLower(feederPercent * 	((Math.abs(triggerAxis.getAsDouble()) > secondThreshold) ? 1 : 0)); // then start feeding 
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
