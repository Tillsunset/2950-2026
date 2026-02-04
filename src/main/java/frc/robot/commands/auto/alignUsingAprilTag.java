package frc.robot.commands.auto;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.driveTrain;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;

public class alignUsingAprilTag extends Command {
	private final driveTrain m_driveTrain;

	private double kPX = 1.5;

	public alignUsingAprilTag(driveTrain driveTrain) {
		m_driveTrain = driveTrain;
		addRequirements(m_driveTrain);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if (LimelightHelpers.getTV("")) {
			Pose3d pose = LimelightHelpers.getTargetPose3d_CameraSpace("");
			System.out.println("found AprilTag");
			System.out.print("distance: ");
			System.out.println(pose.getZ());
			System.out.print("offset: ");
			System.out.println(pose.getX());

			m_driveTrain.setArcade(Math.abs(pose.getZ())/3 - 1, pose.getX() * kPX);
		}
		else {
			System.out.println("not found");
		}
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
