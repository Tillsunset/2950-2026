package frc.robot.commands.flywheel;

import java.util.List;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helper;
import frc.robot.subsystems.flywheel;
import frc.robot.subsystems.conveyor;

import limelight.Limelight;
import limelight.results.RawFiducial;

public class flywheelAutoFeed extends Command {
	private final flywheel m_flywheel;
	private final conveyor m_conveyor;

	private double feederPercent = 0.75;

	private boolean RPMReady = false;

	LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

	Limelight ll = new Limelight("limelight");

	public flywheelAutoFeed(flywheel flywheel, conveyor conveyor) {
		m_flywheel = flywheel;
		addRequirements(m_flywheel);
		m_conveyor = conveyor;
		addRequirements(m_conveyor);

		ll.getSettings()
		.withAprilTagIdFilter(List.of(2, 5, 10, 18, 21, 26))
		.save();
	}

	@Override
	public void initialize() {
		RPMReady = false;
	}

	@Override
	public void execute() {
		// first start flywheel 
		RawFiducial[] raw = ll.getData().getRawFiducials();
		for (RawFiducial object : raw){
			filter.calculate(object.distToCamera);
		}
		double predictedRPM = Helper.rpmFromMeters(filter.lastValue());

		Helper.printRPMDistance(predictedRPM, filter.lastValue());

		m_flywheel.setTargetRPM(predictedRPM);

		if (RPMReady) {
			m_flywheel.setLower(feederPercent);
		}
		else {
			m_flywheel.setLower(0);
			// less then 10% off, start feeding, don't stop until we let go
			RPMReady = Math.abs((m_flywheel.getCurrentRPM() - predictedRPM) / predictedRPM) < 0.1;
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
