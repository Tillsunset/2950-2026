package frc.robot.commands.flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Helper;
import frc.robot.subsystems.swervedrive;

public class flywheelAIM extends Command {
	private final swervedrive m_swervedrive;
	private DoubleSupplier xDoubleSupplier = () -> 0;
	private DoubleSupplier yDoubleSupplier = () -> 0;

	// only P term for now
	private double kP = 0.01;
	private double sign = 1;

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
		m_swervedrive.driveCommand(xDoubleSupplier, yDoubleSupplier, () -> rotationCalc);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
