package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class driveTrain extends SubsystemBase {

	private SparkMax leftLeader = new SparkMax(26, MotorType.kBrushless);
	private WPI_TalonSRX leftFollower = new WPI_TalonSRX(3);

	private SparkMax rightLeader = new SparkMax(25, MotorType.kBrushless);
	private WPI_TalonSRX rightFollower = new WPI_TalonSRX(10);

	public DifferentialDrive neoDrive = new DifferentialDrive(leftLeader, rightLeader);
	public DifferentialDrive cimDrive = new DifferentialDrive(leftFollower, rightFollower);

	public driveTrain() {
		SparkMaxConfig rconfig = new SparkMaxConfig();
		rconfig.inverted(true);
		rconfig.idleMode(IdleMode.kCoast);

		SparkMaxConfig lconfig = new SparkMaxConfig();
		lconfig.inverted(false);
		lconfig.idleMode(IdleMode.kCoast);

		leftLeader.configure(lconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		rightLeader.configure(rconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		leftFollower.setNeutralMode(NeutralMode.Coast);
		rightFollower.setNeutralMode(NeutralMode.Coast);

		rightFollower.setInverted(true);
	}

	public void setTank(double l, double r) {
		neoDrive.tankDrive(l, r);
		cimDrive.tankDrive(l, r);
	}
	
	public void setArcade(double x, double r) {
		neoDrive.arcadeDrive(x, r);
		cimDrive.arcadeDrive(x, r);
	}

	@Override
	public void periodic() {
	}
}
