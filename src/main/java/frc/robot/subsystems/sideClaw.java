package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class sideClaw extends SubsystemBase {

	private SparkMax vertical = new SparkMax(11, MotorType.kBrushless);

	private SparkClosedLoopController clc = vertical.getClosedLoopController();

		private RelativeEncoder encoder = vertical.getEncoder();

	public sideClaw() {
		SparkMaxConfig vConfig = new SparkMaxConfig();
			vConfig.smartCurrentLimit(20)
			.inverted(false)
			.idleMode(IdleMode.kBrake)
			.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(0.01)
				.i(0)
				.d(0)
				.outputRange(-1, 1)
				.feedForward
					.kS(0.03)
					// kV is now in Volts, so we multiply by the nominal voltage (12V)
					.kV(12.0 / 5767)
					.kA(0);

		vertical.configure(vConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
	}

	public void setTargetPosition(double position) {
		clc.setSetpoint(position , ControlType.kPosition);
	}

	
	public void resetEncoder() {
		
		encoder.setPosition(0);
	}
}
