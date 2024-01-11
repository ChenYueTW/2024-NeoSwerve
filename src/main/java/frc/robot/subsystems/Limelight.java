package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.BetterLimelight;
import frc.robot.lib.IDashboardProvider;

public class Limelight extends SubsystemBase implements IDashboardProvider {
    private final BetterLimelight limelight;
    private double distanceToGoalVerticalMeters;
    private double distanceToGoalHorizontalMeters;

    public Limelight() {
        this.registerDashboard();
        this.limelight = new BetterLimelight("limelight");
    }

    public double getDistanceToGoalVerticalMeters() {
        this.distanceToGoalVerticalMeters = this.limelight.getDistanceToGoalVerticalMeters();
        return this.distanceToGoalVerticalMeters;
    }

    public double getDistanceToGoalHorizontalMeters(double distanceToGoalVerticalMeters) {
        this.distanceToGoalHorizontalMeters = this.limelight.getDistanceToGoalHorizontalMeters(distanceToGoalVerticalMeters);
        return this.distanceToGoalHorizontalMeters;
    }

    public double getAprilTagId() {
        return this.limelight.getAprilTagId();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Vertical", this.distanceToGoalVerticalMeters);
        SmartDashboard.putNumber("Horizontal", this.distanceToGoalHorizontalMeters);
        SmartDashboard.putNumber("AprilTag", this.getAprilTagId());
    }
}
