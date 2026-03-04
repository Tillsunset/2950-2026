package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class flywheel extends SubsystemBase {

	private SparkFlex leftVortex = new SparkFlex(23, MotorType.kBrushless);
	private SparkFlex rightVortex = new SparkFlex(22, MotorType.kBrushless);
	private SparkMax frontWheel = new SparkMax(15, MotorType.kBrushless);
	private SparkMax backWheel = new SparkMax(2, MotorType.kBrushless);
	private SparkMax conveyor = new SparkMax(5, MotorType.kBrushed);

	private SparkClosedLoopController closedLoopController = leftVortex.getClosedLoopController();

	public flywheel() {
		SparkMaxConfig fConfig = new SparkMaxConfig();
			fConfig.inverted(true)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(40);

		SparkMaxConfig bConfig = new SparkMaxConfig();
			bConfig.apply(fConfig)
				.inverted(false);
		
		SparkMaxConfig wheelConfig = new SparkMaxConfig();
			wheelConfig.apply(fConfig)
				.inverted(false);

		frontWheel.configure(fConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		backWheel.configure(bConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		conveyor.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


		SparkFlexConfig lVortexConfig = new SparkFlexConfig();
			lVortexConfig.smartCurrentLimit(80)
			.inverted(true);

			lVortexConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				// Set PID values for Velocity control. We don't need to pass a closed
				// loop slot, as it will default to slot 0.
				.p(0.0001)
				.i(0)
				.d(0)
				.outputRange(0, 1)
				.feedForward
					.kS(0.03)
					// kV is now in Volts, so we multiply by the nominal voltage (12V)
					.kV(12.0 / 5767)
					.kA(12/(4000/1.0));

			lVortexConfig.closedLoop.maxMotion
				// Set MAXMotion parameters for position control. We don't need to pass
				// a closed loop slot, as it will default to slot 0.
				.maxAcceleration(2000);

		SparkFlexConfig rVortexConfig = new SparkFlexConfig();
			rVortexConfig.apply(lVortexConfig)
				.follow(leftVortex, true);

		leftVortex.configure(lVortexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		rightVortex.configure(rVortexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
	}

	public void setLower(double percent) {
		frontWheel.set(percent);
		backWheel.set(percent);
		conveyor.set(percent);
	}

	public void setTargetRPM(double rpm) {
		closedLoopController.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
	}
}
