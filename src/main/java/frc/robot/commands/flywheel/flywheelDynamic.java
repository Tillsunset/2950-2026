package frc.robot.commands.flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Helper;
import frc.robot.subsystems.flywheel;

public class flywheelDynamic extends Command {
	private final flywheel m_flywheel;
	private DoubleSupplier triggerAxis;

	private double feederPercent = 1;

	private double firstThreshold = 0.1;
	private double secondThreshold = 0.9;

	public flywheelDynamic(flywheel flywheel, CommandXboxController x) {
		triggerAxis = x::getLeftTriggerAxis;
		m_flywheel = flywheel;
		addRequirements(m_flywheel);
	}

	@Override
	public void initialize() {
		Helper.resetFilters();
		m_flywheel.setLower(-.1);
	}

	@Override
	public void execute() {
		// first start flywheel 
		if (Math.abs(triggerAxis.getAsDouble()) > firstThreshold) {

			m_flywheel.leftVortex.set(1);
		}
		else {
			m_flywheel.leftVortex.set(0);
		}

		m_flywheel.setLower(feederPercent *    ((Math.abs(triggerAxis.getAsDouble()) > secondThreshold) ? 1 : 0)); // then start feeding 
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
