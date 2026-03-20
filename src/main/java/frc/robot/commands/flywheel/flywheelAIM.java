package frc.robot.commands.flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helper;
import frc.robot.subsystems.swervedrive;

public class flywheelAIM extends Command {
	private final swervedrive m_swervedrive;

	// only P term for now
	private double kP = 0.05;
	private double sign = -1;

	public flywheelAIM(swervedrive swervedrive) {
		m_swervedrive = swervedrive;
		addRequirements(m_swervedrive);
	}

	@Override
	public void initialize() {
		Helper.resetFilters();
	}

	@Override
	public void execute() {
		Helper.updateFilters();
		double xDist = Helper.getAprilTagAim();

		double rotationCalc = sign * (xDist * kP);

		Helper.printRPMDistance(xDist, rotationCalc);

		// m_swervedrive.driveCommand(xDoubleSupplier, yDoubleSupplier, wrapper);
		m_swervedrive.drive(new Translation2d(), rotationCalc, false);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
