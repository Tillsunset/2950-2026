package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helper;
import frc.robot.subsystems.conveyor;
import frc.robot.subsystems.flywheel;

public class flywheelStatic extends Command {
	private final flywheel m_flywheel;
	private final conveyor m_conveyor;

	private double conveyorPercent = 1;

	private double feederPercent = 1;
	private double flywheelRPM = 1500;

	private boolean RPMReady = false;

	public flywheelStatic(flywheel flywheel, conveyor conveyor, double staticSetpoint) {
		m_flywheel = flywheel;
		addRequirements(m_flywheel);
		m_conveyor = conveyor;
		addRequirements(m_conveyor);

		flywheelRPM = staticSetpoint;
	}

	@Override
	public void initialize() {
		m_flywheel.setTargetRPM(flywheelRPM);
		m_flywheel.setLower(-.1);
		Helper.resetFilters();
		RPMReady = false;
	}

	@Override
	public void execute() {
		Helper.updateFilters();
		double distance = Helper.getAprilTagDist();

		Helper.printRPMDistance(flywheelRPM, distance);

		if (RPMReady) {
			m_flywheel.setLower(feederPercent);
			m_conveyor.setConveyer(conveyorPercent);
		}
		else {
			// less then 10% off, start feeding, don't stop until we let go
			RPMReady = Math.abs((m_flywheel.getCurrentRPM() - flywheelRPM) / flywheelRPM) < 0.1;
		}
	}

	@Override
	public void end(boolean interrupted) {
		m_flywheel.setTargetRPM(0);
		m_flywheel.setLower(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
