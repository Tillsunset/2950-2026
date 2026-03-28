package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class intake extends SubsystemBase {

	private SparkMax leftArm = new SparkMax(16, MotorType.kBrushless);
	private SparkMax rightArm = new SparkMax(17, MotorType.kBrushless);
	public SparkMax wheel = new SparkMax(4, MotorType.kBrushless);

	private SparkClosedLoopController leftClosedLoopController = leftArm.getClosedLoopController();
	private SparkClosedLoopController rightClosedLoopController = rightArm.getClosedLoopController();

	private RelativeEncoder leftEncoder = leftArm.getEncoder();
	private RelativeEncoder rightEncoder = rightArm.getEncoder();

	private double feedForward = 0.0;

	public intake() {

		SparkMaxConfig rConfig = new SparkMaxConfig();
		rConfig.inverted(true)
			.idleMode(IdleMode.kBrake)
			.smartCurrentLimit(20)
			.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(0.025)
				.d(0)
				.outputRange(-1, 1);


		SparkMaxConfig lConfig = new SparkMaxConfig();
		lConfig.apply(rConfig)
		.inverted(false);
			// .follow(17,true);
			// no follow due to 'loose' arm

		
		SparkMaxConfig wheelConfig = new SparkMaxConfig();
		wheelConfig
			.smartCurrentLimit(60)
			.inverted(true)
			.idleMode(IdleMode.kCoast);

		leftArm.configure(lConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		rightArm.configure(rConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		wheel.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

	}

	@Override
	public void periodic() {
	}

	public void resetEncoder() {
		
		leftEncoder.setPosition(0);
		rightEncoder.setPosition(0);
	}

	public void updateTargetAngle(double target) {
		// no follow due to 'loose' arm, separate CLC
		leftClosedLoopController.setSetpoint(target, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward, ArbFFUnits.kPercentOut);
		rightClosedLoopController.setSetpoint(target, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward, ArbFFUnits.kPercentOut);
	}

	public void setWheel(double percent) {
		wheel.set(percent);
	}
}
