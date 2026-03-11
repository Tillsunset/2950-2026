package frc.robot.commands;

import frc.robot.subsystems.sideClaw;
import edu.wpi.first.wpilibj2.command.Command;

public class clawControl extends Command {
	private final sideClaw m_sideClaw;
	private double m_position = 0;

	public clawControl(sideClaw sideClaw, double position){
		m_sideClaw = sideClaw;
		m_position = position;
		addRequirements(m_sideClaw);
	}

	@Override
	public void initialize() {
		m_sideClaw.setTargetPosition(m_position);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}