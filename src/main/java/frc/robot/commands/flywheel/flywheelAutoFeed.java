package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helper;
import frc.robot.subsystems.flywheel;
import frc.robot.subsystems.conveyor;

public class flywheelAutoFeed extends Command {
	private final flywheel m_flywheel;
	private final conveyor m_conveyor;

	private double feederPercent = 0.75;
	private double conveyorPercent = 0.5;

	private boolean RPMReady = false;

	public flywheelAutoFeed(flywheel flywheel, conveyor conveyor) {
		m_flywheel = flywheel;
		addRequirements(m_flywheel);
		m_conveyor = conveyor;
		addRequirements(m_conveyor);
	}

	@Override
	public void initialize() {
		RPMReady = false;
		m_flywheel.setLower(0);
		m_conveyor.setConveyer(0);
		Helper.resetFilters();
	}

	@Override
	public void execute() {
		Helper.updateFilters();
		double distance = Helper.getAprilTagDist();
		double predictedRPM = Helper.rpmFromMeters(distance);

		Helper.printRPMDistance(predictedRPM, distance);
		m_flywheel.setTargetRPM(predictedRPM);

		if (RPMReady) {
			m_flywheel.setLower(feederPercent);
			m_conveyor.setConveyer(conveyorPercent);
		}
		else {
			// less then 10% off, start feeding, don't stop until we let go
			RPMReady = Math.abs((m_flywheel.getCurrentRPM() - predictedRPM) / predictedRPM) < 0.1;
		}
	}

	@Override
	public void end(boolean interrupted) {
		m_flywheel.setTargetRPM(0);
		m_flywheel.setLower(0);
		m_conveyor.setConveyer(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
