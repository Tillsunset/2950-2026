package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Pose;

public class driveTrain extends SubsystemBase {

	/*****************Odometry variables**********************/
	public static double motorRPMToVelocity = 3.1415 * 6./(39.37 * 60. * 8.45);
	public static double motorPowerToVelocity = 5880. * motorRPMToVelocity;
	public double posX, posY;
	public double linearVel, prevLinearVel;

	private double prevLeftWheelVel = 0.0, prevRightWheelVel = 0.0;  // Previous wheel velocities for trapezoidal integration
	private double leftVel = 0.0, rightVel = 0.0;  // Low-pass filtered velocities

	private static final double ALPHA = 0.8;  // Low-pass filter coefficient

	private static final double BETA = 0.8;  // Low-pass filter coefficient
	private static final double KAPPA = 0.8;  // Velocity Bias over motor Power
	public double yaw; // Smoothed gyro heading

	private double dt = 0.02;

	ADIS16470_IMU IMU = new ADIS16470_IMU();
	/*****************Odometry variables**********************/

	/*******************Stanley Control variables**********************/
	private double steerKp = .175; // tune heading error
	private double k = 1; // tune cross error
	private double maxSteer = Math.toRadians(45);

	double motorPower = 0.225;
	double zRotation = 0;
	double xSpeed = 0;
	/*******************Stanley Control variables**********************/

	/*******************Interpolation variables**********************/
	public int numPoints = 6;

	public Pose current = new Pose();
	public Pose goal = new Pose();

	private static final double DELTA = 0.8;

	public List<Pose> waypoints = new ArrayList<>();
	/*******************Interpolation variables**********************/

	private SparkMax driveFR = new SparkMax(16, MotorType.kBrushless);
	private SparkMax driveBR = new SparkMax(20, MotorType.kBrushless);

	private SparkMax driveFL = new SparkMax(17, MotorType.kBrushless);
	private SparkMax driveBL = new SparkMax(3, MotorType.kBrushless);

	public DifferentialDrive driveBase = new DifferentialDrive(driveFL, driveFR);

	private RelativeEncoder right = driveFR.getEncoder();
	private RelativeEncoder left = driveFL.getEncoder();


	public driveTrain() {
		SparkMaxConfig globalConfig = new SparkMaxConfig();
		SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
		SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
		SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

		globalConfig
			.smartCurrentLimit(50)
			.inverted(true)
			.idleMode(IdleMode.kBrake);

		leftFollowerConfig
			.apply(globalConfig)
			.follow(driveFL);

		rightLeaderConfig
			.apply(globalConfig)
			.inverted(false);

		rightFollowerConfig
			.apply(rightLeaderConfig)
			.follow(driveFR);

		driveFR.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		driveBR.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		driveFL.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		driveBL.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		
		IMU.calibrate();
		resetPos();
		for (int i = 0; i < numPoints; i++) {
			waypoints.add(new Pose(0, 0, 0)); // Pre-allocate memory
		}
	}

	@Override
	public void periodic() {
		// printGoalMotor();
		// printPosVelHead();
	}

	public void printPosVelHead() {
		System.out.print("X position: ");
		System.out.printf("%.2f\n", posX);

		System.out.print("Y position: ");
		System.out.printf("%.2f\n", posY);

		// System.out.print("linear veloctiy: ");
		// System.out.printf("%.2f\n", linearVel);

		System.out.print("heading: ");
		System.out.printf("%.2f\n", yaw);
	}

	public void printGoalMotor() {
		System.out.print("xSpeed: ");
		System.out.printf("%.2f\n", xSpeed);

		System.out.print("zRotation: ");
		System.out.printf("%.2f\n", zRotation);

		System.out.print("Goal: ");
		System.out.printf("%.2f %.2f %.2f\n", goal.x, goal.y, goal.yaw);
	}

	private double distance(double x, double y) {
		return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
	}

	private double clamp(double upper, double value, double lower) {
		return Math.max(lower, Math.min(upper, value));
	}

	public boolean getFinishedGoal() {
		double directionX = Math.cos(goal.yaw);
		
		double displacementX = posX - goal.x;
		
		// Compute projection onto the goal direction
		double projection = displacementX * directionX;
		
		// If projection is positive, we are on the correct side (past the goal)
		return projection > 0;
	}

	public void updateOdometry() {
		// Low Pass Filter Yaw and Wheel selocity signals
		yaw = BETA * yaw + (1 - BETA) * Math.toRadians(IMU.getAngle());

		leftVel = ALPHA * leftVel + (1 - ALPHA) * (Math.abs(left.getVelocity()) * motorRPMToVelocity * KAPPA + Math.abs(driveFL.get()) * (1 - KAPPA) * motorPowerToVelocity);
		rightVel = ALPHA * rightVel + (1 - ALPHA) * (Math.abs(right.getVelocity()) * motorRPMToVelocity * KAPPA + Math.abs(driveFR.get()) * (1 - KAPPA) * motorPowerToVelocity);

		// Compute using trapezoidal integration
		double linearVel = ((prevLeftWheelVel + leftVel) + (prevRightWheelVel + rightVel)) / 4.0;

		posX += 0.5 * (linearVel + prevLinearVel) * Math.cos(yaw) * dt;
		posY += 0.5 * (linearVel + prevLinearVel) * Math.sin(yaw) * dt;

		// Store previous wheel velocities for next step
		prevLeftWheelVel = leftVel;
		prevRightWheelVel = rightVel;
		prevLinearVel = linearVel;
	}

	public void resetPos() {
		posX = 0.0;
		posY = 0.0;
		yaw = 0.0;
		linearVel = 0.0;
		IMU.reset();
	}

	public void resetWaypoints() {
		for (int i = 0; i < numPoints; i++) {
			waypoints.get(i).x = 0;
			waypoints.get(i).y = 0;
			waypoints.get(i).yaw = 0;
		}
		
		current.x = 0;
		current.y = 0;
		current.yaw = 0;

		goal.x = 0;
		goal.y = 0;
		goal.yaw = 0;
	}

	public void updateWaypoints () {
		if (LimelightHelpers.getTV("")) {
			resetPos();

			current.x = 0;
			current.y = -.1;
			current.yaw = 0;


			Pose3d pose = LimelightHelpers.getTargetPose3d_CameraSpace("");
			
			// goal.x = pose.getZ();
			// goal.y = pose.getX()+ 0.1;
			// goal.yaw = -pose.getRotation().getY();

			goal.x = DELTA * goal.x + (1 - DELTA) * (pose.getZ());
			goal.y = DELTA * goal.y + (1 - DELTA) * (pose.getX());
			goal.yaw = DELTA * goal.yaw + (1 - DELTA) * ( -pose.getRotation().getY());

			interpolateAlignedPoints(current, goal, numPoints);
		}
		else {
			current.x = posX;
			current.y = posY;
			current.yaw = yaw;
		}
	}

	public void updateMotors() {
		if (waypoints.get(numPoints - 1).x != 0) {
			double steer = computeControl(current, waypoints);
			computeMotorPower(motorPower, steer);
		}
	}

	private double computeControl(Pose current, List<Pose> waypoints) {
		double minDist = Double.MAX_VALUE;
		int nearestIdx = 0;
		
		for (int i = 0; i < waypoints.size(); i++) {
			double dist = distance(waypoints.get(i).x - current.x, waypoints.get(i).y - current.y);
			if (dist < minDist) {
				minDist = dist;
				nearestIdx = i;
			}
		}
		
		double targetX = waypoints.get(nearestIdx).x;
		double targetY = waypoints.get(nearestIdx).y;
		
		double pathYaw = Math.atan2(
			waypoints.get(Math.min(nearestIdx + 1, waypoints.size() - 1)).y - targetY,
			waypoints.get(Math.min(nearestIdx + 1, waypoints.size() - 1)).x - targetX
		);
		
		double crossTrackError = Math.sin(pathYaw) * (current.x - targetX) - Math.cos(pathYaw) * (current.y - targetY);

		double headingError = pathYaw - current.yaw;
		headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
		
		double stanley_steer = headingError + Math.atan2(k * crossTrackError, (motorPower * driveTrain.motorPowerToVelocity * 0.1 + linearVel * 0.9));
		return clamp(maxSteer, stanley_steer, -maxSteer);
	}

	private void computeMotorPower(double motorPower, double steer) {
		zRotation = steerKp * steer;
		xSpeed = motorPower;

		driveBase.arcadeDrive(-xSpeed, zRotation, false);
	}

	public void interpolateAlignedPoints(Pose current, Pose goal, int numPoints) {
		double directionX = Math.cos(goal.yaw);
		double directionY = Math.sin(goal.yaw);
		
		// Project current position onto the goal's tangent line
		double displacementX = current.x - goal.x;
		double displacementY = current.y - goal.y;
		double projectionLength = displacementX * directionX + displacementY * directionY;
		double projectedStartX = goal.x + projectionLength * directionX;
		double projectedStartY = goal.y + projectionLength * directionY;
		
		// Generate evenly spaced points from current to goal along the tangent line
		double totalDistance = Math.sqrt(Math.pow(goal.x - projectedStartX, 2) + Math.pow(goal.y - projectedStartY, 2));
		double spacing = totalDistance / (numPoints - 1);
		
		for (int i = 0; i < numPoints; i++) {
			waypoints.get(i).x = projectedStartX + i * spacing * directionX;
			waypoints.get(i).y = projectedStartY + i * spacing * directionY;
			waypoints.get(i).yaw = goal.yaw;
		}
	}
}
