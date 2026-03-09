package frc.robot.commands;

import frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;

public class climberControl extends Command {
	private final climber m_climber;
	private double m_position = 0;

	public climberControl(climber climber, double position){
		m_climber = climber;
		m_position = position;
		addRequirements(m_climber);
	}

	@Override
	public void initialize() {
		m_climber.setTargetPosition(m_position);
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