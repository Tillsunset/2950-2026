package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.conveyor;

public class conveyorControl extends Command {
	private final conveyor m_conveyor;

	private DoubleSupplier triggerLAxis;
	private DoubleSupplier triggerRAxis;

	private double intakeThreshold = 0.5;
	private double flywheelThreshold = 0.8;
	
	private double conveyorPercent = 1;

	public conveyorControl(conveyor conveyor, CommandXboxController x) {
		triggerLAxis = x::getLeftTriggerAxis;
		triggerRAxis = x::getRightTriggerAxis;
		m_conveyor = conveyor;
		addRequirements(m_conveyor);
	}

	
	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if ((Math.abs(triggerLAxis.getAsDouble()) > flywheelThreshold) ||
			(Math.abs(triggerRAxis.getAsDouble()) > intakeThreshold)) {
				// m_conveyor.setConveyer(conveyorPercent);
		}
		else {
			m_conveyor.setConveyer(0);
		}
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
