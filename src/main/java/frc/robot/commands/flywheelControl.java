package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Helper;
import frc.robot.subsystems.flywheel;

import limelight.Limelight;
import limelight.results.RawFiducial;

public class flywheelControl extends Command {
	private final flywheel m_flywheel;
	private DoubleSupplier triggerAxis;

	private double feederPercent = 0.5;

	private double firstThreshold = 0.2;
	private double secondThreshold = 0.8;

	LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

	Limelight ll = new Limelight("limelight");

	public flywheelControl(flywheel flywheel, CommandXboxController x) {
		triggerAxis = x::getLeftTriggerAxis;
		m_flywheel = flywheel;
		addRequirements(m_flywheel);

		ll.getSettings()
		.withAprilTagIdFilter(List.of(2, 5, 10, 18, 21, 26))
		.save();
	}

	@Override
	public void initialize() {
		// Time constant is 0.1 seconds
		// Period is 0.02 seconds - this is the standard FRC main loop period
	}

	@Override
	public void execute() {
		// first start flywheel 
		if (Math.abs(triggerAxis.getAsDouble()) > firstThreshold) {
			RawFiducial[] raw = ll.getData().getRawFiducials();
			for (RawFiducial object : raw){
				filter.calculate(object.distToCamera);
			}
			double predictedRPM = Helper.rpmFromInches(filter.lastValue());
			m_flywheel.setTargetRPM(predictedRPM);
		}
		else {
			m_flywheel.setTargetRPM(0);
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
