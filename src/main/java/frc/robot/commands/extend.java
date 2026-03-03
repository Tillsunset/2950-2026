package frc.robot.commands;

import frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class extend extends Command {
	private final climber m_climber;
	private double position = 20;

	public extend(climber climber){
		m_climber = climber;
		addRequirements(m_climber);
	}

	@Override
	public void initialize() {
		m_climber.setTargetPosition(position);
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