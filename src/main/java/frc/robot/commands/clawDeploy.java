package frc.robot.commands;

import frc.robot.subsystems.sideClaw;
import edu.wpi.first.wpilibj2.command.Command;

public class clawDeploy extends Command {
	private final sideClaw m_sideClaw;
	private double position = 80;

	public clawDeploy(sideClaw sideClaw){
		m_sideClaw = sideClaw;
		addRequirements(m_sideClaw);
	}

	@Override
	public void initialize() {
		m_sideClaw.setTargetPosition(position);
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