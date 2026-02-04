package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;

public class RobotContainer {

	private CommandXboxController purple = new CommandXboxController(0);
	private Trigger rb = purple.rightBumper(); 

	private driveTrain m_driveTrain = new driveTrain();
	
	private driveTank m_driveTank = new driveTank(m_driveTrain, purple.getHID());

	private alignUsingAprilTag m_align = new alignUsingAprilTag(m_driveTrain);

	public RobotContainer() {

		m_driveTrain.setDefaultCommand(m_driveTank);

		configureBindings();
	}
	
	private void configureBindings() {
		rb.whileTrue(m_align);
	}

	public Command getAutonomousCommand() {
	    return null;
	}
}
