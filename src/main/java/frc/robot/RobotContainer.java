package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

	private CommandXboxController controller0 = new CommandXboxController(0);

	public intake m_intake = new intake();
	private intakeControl m_intakeControl = new intakeControl(m_intake, controller0);

	private flywheel m_flywheel = new flywheel();
	private flywheelControl m_flywheelControl = new flywheelControl(m_flywheel, controller0);
	
	// public climber m_climber = new climber();
	// private climberControl m_Retract = new climberControl(m_climber, 0);
	// private climberControl m_Extend = new climberControl(m_climber, 80);
	// private Trigger dpadUp = controller0.povUp();
	// private Trigger dpadDown = controller0.povDown();

	// public sideClaw m_sideClaw = new sideClaw();
	// private clawControl m_clawStow = new clawControl(m_sideClaw, 0);
	// private clawControl m_clawDeploy = new clawControl(m_sideClaw, 80);
	// private Trigger dpadLeft = controller0.povLeft();
	// private Trigger dpadRight = controller0.povRight();

	private CommandXboxController green = new CommandXboxController(1);

	public RobotContainer() {
		m_intake.setDefaultCommand(m_intakeControl);
		m_flywheel.setDefaultCommand(m_flywheelControl);

		configureBindings();
	}
	
	private void configureBindings() {
		// dpadUp.onTrue(m_Extend);
		// dpadDown.onTrue(m_Retract);
		// dpadLeft.onTrue(m_clawStow);
		// dpadRight.onTrue(m_clawDeploy);
	}

	public Command getAutonomousCommand() {
	    return null;
	}
}
