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

public class conveyor extends SubsystemBase {

	private SparkMax conveyorMotor = new SparkMax(21, MotorType.kBrushed);
    private SparkMax spindexerMotor = new SparkMax(11, MotorType.kBrushless);

	public conveyor() {
		SparkMaxConfig Config = new SparkMaxConfig();
		Config.inverted(true)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(40);

				conveyorMotor.configure(Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	public void spindexer() {
		SparkMaxConfig Config = new SparkMaxConfig();
		Config.inverted(true)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(40);

				conveyorMotor.configure(Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}


	@Override
	public void periodic() {
	}

	public void setConveyer( double percent){
		conveyorMotor.set(percent);
	    spindexerMotor.set(percent);
	}
}
