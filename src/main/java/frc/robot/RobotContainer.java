// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive;
import java.io.File;
import swervelib.SwerveInputStream;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

	// Replace with CommandPS4Controller or CommandJoystick if needed
	final CommandXboxController controller0 = new CommandXboxController(0);

	public intake m_intake = new intake();
	private intakeControl m_intakeControl = new intakeControl(m_intake, controller0);

	private flywheel m_flywheel = new flywheel();
	private flywheelControl m_flywheelControl = new flywheelControl(m_flywheel, controller0);

	private conveyor m_conveyor = new conveyor();
	private conveyorControl m_conveyorControl = new conveyorControl(m_conveyor, controller0);

	// The robot's subsystems and commands are defined here...
	private final swervedrive drivebase = new swervedrive(new File(Filesystem.getDeployDirectory(),
			"swerve"));

	// Establish a Sendable Chooser that will be able to be sent to the
	// SmartDashboard, allowing selection of desired auto
	private final SendableChooser<Command> autoChooser = new SendableChooser<>();

	/**
	 * Converts driver input into a field-relative ChassisSpeeds that is controlled
	 * by angular velocity.
	 */
	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
			() -> controller0.getLeftY() * -1,
			() -> controller0.getLeftX() * -1)
			.withControllerRotationAxis(controller0::getRightX)
			.deadband(OperatorConstants.DEADBAND)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a fieldRelative
	 * input stream.
	 */
	SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(controller0::getRightX,
			controller0::getRightY)
			.headingWhile(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a robotRelative
	 * input stream.
	 */
	SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
			.allianceRelativeControl(false);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		m_intake.setDefaultCommand(m_intakeControl);
		m_flywheel.setDefaultCommand(m_flywheelControl);
		
		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);

		// Set the default auto (do nothing)
		autoChooser.setDefaultOption("Do Nothing", Commands.runOnce(drivebase::zeroGyroWithAlliance)
				.andThen(Commands.none()));

		// Add a simple auto option to have the robot drive forward for 1 second then
		// stop
		autoChooser.addOption("Drive Forward", Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2)
				.andThen(drivebase.driveForward().withTimeout(1)));
		// Put the autoChooser on the SmartDashboard
		SmartDashboard.putData("Auto Chooser", autoChooser);

		if (autoChooser.getSelected() == null) {
			RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
		}
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary predicate, or via the
	 * named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
	 * for
	 * {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
	 * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
	 * Flight joysticks}.
	 */
	private void configureBindings() {
		Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
		Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

		drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
		flywheel.setDefaultCommand(flywheelControl);
		intake.setDefaultCommand(intakeControl);
		conveyor.setDefaultCommand(conveyorControl);

		controller0.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
		controller0.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// Pass in the selected auto from the SmartDashboard as our desired autnomous
		// commmand
		return autoChooser.getSelected();
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}
}