package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

	private CommandXboxController purple = new CommandXboxController(0);

	public intake m_intake = new intake();
	private intakeControl m_intakeControl = new intakeControl(m_intake,purple);

	private flywheel m_flywheel = new flywheel();
	private flywheelControl m_flywheelControl = new flywheelControl(m_flywheel, purple);
	
	private climber m_Climber = new climber();
	private Trigger dpadUp = purple.povUp();
	private Trigger dpadDown = purple.povDown();
	private extend m_Extend = new extend(m_Climber);
	private retract m_Retract = new retract(m_Climber);

	public RobotContainer() {

		m_intake.setDefaultCommand(m_intakeControl);
		m_flywheel.setDefaultCommand(m_flywheelControl);

		configureBindings();
	}
	
	private void configureBindings() {
		dpadUp.onTrue(m_Extend);
		dpadDown.onTrue(m_Retract);
	}

	public Command getAutonomousCommand() {
	    return null;
	}
}
