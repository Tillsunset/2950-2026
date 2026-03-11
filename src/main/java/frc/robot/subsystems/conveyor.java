package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.flywheel;

public class conveyor {

	private SparkMax conveyorMotor = new SparkMax(21, MotorType.kBrushed);

	public conveyor() {
		SparkMaxConfig Config = new SparkMaxConfig();
			fConfig.inverted(true)
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(40);

		conveyor.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
	}

	public void setConveyer( double percent){
		conveyor.set(percent);
	}
}
