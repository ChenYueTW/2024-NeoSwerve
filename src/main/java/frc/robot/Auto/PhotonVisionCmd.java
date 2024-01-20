package frc.robot.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;

public class PhotonVisionCmd extends Command {
	private final SwerveSubsystem swerveSubsystem;
	private final PhotonVision photoVision;
	private final PIDController translationPid;
	private final PIDController rotationPid;

	public PhotonVisionCmd(SwerveSubsystem swerveSubsystem, PhotonVision photoVision) {
		this.swerveSubsystem = swerveSubsystem;
		this.photoVision = photoVision;
		this.translationPid = new PIDController(0.06, 0.0, 0.0);
		this.rotationPid = new PIDController(0.3, 0.0, 0.0);

		addRequirements(this.swerveSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		boolean hasTargets = this.photoVision.hasTargets();
		double distanceToGoalVerticalMeters = this.photoVision.getDistanceToGoalVerticalMeters();
		double distanceToGoalHorizontalMeters = this.photoVision.getDistanceToGoalHorizontalMeters(distanceToGoalVerticalMeters);

		double distance = 0.900666419935816;
		double horizontal = 0; // -0.19

		double verticalSpeed = MathUtil.applyDeadband(
			this.translationPid.calculate(distanceToGoalVerticalMeters, distance), -0.05) * LimelightConstants.VERTICAL_MAX_SPEED;
		double rotationSpeed = MathUtil.applyDeadband(
			this.rotationPid.calculate(distanceToGoalHorizontalMeters, horizontal), 0.1) * LimelightConstants.HORIZONTAL_MAX_SPEED;

		SmartDashboard.putNumber("verticalSpeed", verticalSpeed);
		SmartDashboard.putNumber("rotationSpeed", rotationSpeed);
		
		if (verticalSpeed < 0 && hasTargets) {
			this.swerveSubsystem.driveSwerve(-verticalSpeed, 0.0, rotationSpeed, LimelightConstants.gyroField);
		} else {
			this.swerveSubsystem.driveSwerve(0.0, 0.0, rotationSpeed, LimelightConstants.gyroField);
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
