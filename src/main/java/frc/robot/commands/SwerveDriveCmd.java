package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;

public class SwerveDriveCmd extends Command {
	private final SwerveSubsystem swerveSubsystem;
	private final XboxController controller;

	public SwerveDriveCmd(SwerveSubsystem swerveSubsystem, XboxController controller) {
		this.swerveSubsystem = swerveSubsystem;
		this.controller = controller;
		addRequirements(this.swerveSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double brakes = 1 - MathUtil.applyDeadband(this.controller.getLeftTriggerAxis(), Constants.DEAD_BAND);
		double xSpeed = -MathUtil.applyDeadband(this.controller.getLeftY(), Constants.DEAD_BAND) * Constants.MAX_SPEED * brakes;
		double ySpeed = -MathUtil.applyDeadband(this.controller.getLeftX(), Constants.DEAD_BAND) * Constants.MAX_SPEED * brakes;
		double rotation = -MathUtil.applyDeadband(this.controller.getRightX(), Constants.DEAD_BAND) * Constants.MAX_ANGULAR_SPEED * brakes;

		if (this.controller.getAButton()) {
			this.swerveSubsystem.resetGyro();
		} else if (this.controller.getYButton()) {
			this.swerveSubsystem.resetPose(new Pose2d(2, 7, new Rotation2d()));
		}

		this.swerveSubsystem.driveSwerve(xSpeed, ySpeed, rotation, Constants.gyroField);
	}

	@Override
	public void end(boolean interrupted) {
		this.swerveSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
