package frc.robot.commands.flywheel;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helper;
import frc.robot.subsystems.flywheel;

import limelight.Limelight;
import limelight.results.RawFiducial;

public class flywheelStatic extends Command {
	private final flywheel m_flywheel;

	private double feederPercent = 0.75;
	private double flywheelRPM = 2400;

	LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

	Limelight ll = new Limelight("limelight");

	public flywheelStatic(flywheel flywheel, double staticSetpoint) {
		m_flywheel = flywheel;
		addRequirements(m_flywheel);

		flywheelRPM = staticSetpoint;
	}

	@Override
	public void initialize() {
		m_flywheel.setTargetRPM(flywheelRPM);
		m_flywheel.setLower(feederPercent);
	}

	@Override
	public void execute() {
		RawFiducial[] raw = ll.getData().getRawFiducials();
		for (RawFiducial object : raw){
			filter.calculate(object.distToCamera);
		}

		Helper.printRPMDistance(flywheelRPM, filter.lastValue());
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
