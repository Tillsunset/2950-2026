package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class conveyor extends SubsystemBase {

	private SparkMax conveyorMotor = new SparkMax(21, MotorType.kBrushless);
    private SparkMax spindexerMotor = new SparkMax(5, MotorType.kBrushed);

	public conveyor() {
		SparkMaxConfig Config = new SparkMaxConfig();
		Config.inverted(false)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(20);

		conveyorMotor.configure(Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		SparkMaxConfig spinConfig = new SparkMaxConfig();
			spinConfig.apply(Config)
			.inverted(true)
			.smartCurrentLimit(40);
		spindexerMotor.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
	}

	public void setConveyer( double percent){
		conveyorMotor.set(percent);
	    spindexerMotor.set(percent);
	}
}
