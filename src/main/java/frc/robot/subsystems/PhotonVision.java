package frc.robot.subsystems;


import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.lib.IDashboardProvider;

public class PhotonVision extends SubsystemBase implements IDashboardProvider {
    private final PhotonCamera camera;

    public PhotonVision() {
        this.registerDashboard();
        this.camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    }

    public double getDistanceToGoalVerticalMeters() {
        double verticalOffset = this.getPitch();

        double mountAngleDeg = LimelightConstants.MOUNT_ANGLE_DEG;
        double lensHeightMeters = LimelightConstants.LENS_HEIGHT_METERS;
        double goalHeightMeters = LimelightConstants.GOAL_HEIGHT_METERS;

        double angleToGoalDeg = mountAngleDeg + verticalOffset;
        double angleToGoalRad = angleToGoalDeg * (Math.PI / 180.0);

        if (this.hasTargets()) {
            return Math.abs((goalHeightMeters - lensHeightMeters) / Math.tan(angleToGoalRad));
        } else {
            return 0;
        }
    }

    public double getDistanceToGoalHorizontalMeters(double distanceToGoalVerticalMeters) {
        if (distanceToGoalVerticalMeters == -1) {
            distanceToGoalVerticalMeters = this.getDistanceToGoalVerticalMeters();
        }
        double horizontalOffset = this.getYaw();

        double horizontalOffsetRad = horizontalOffset * (Math.PI / 180.0);

        if (this.hasTargets()) {
            return (Math.tan(horizontalOffsetRad) * distanceToGoalVerticalMeters) - LimelightConstants.HORIZONTAL_OFFSET_METERS;
        } else {
            return 0;
        }
    }

    public boolean hasTargets() {
        return this.camera.getLatestResult().hasTargets();
    }

    public double getPitch() {
        var result = this.camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getPitch();
        } else {
            return 0;
        }
    }

    public double getYaw() {
        var result = this.camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        } else {
            return 0;
        }
    }
    @Override
    public void putDashboard() {
        SmartDashboard.putBoolean("HasCam", this.hasTargets());
        SmartDashboard.putNumber("Pitch", this.getPitch());
        SmartDashboard.putNumber("Yaw", this.getYaw());
        SmartDashboard.putNumber("VerticalMeter", this.getDistanceToGoalVerticalMeters());
        SmartDashboard.putNumber("Horizontal", this.getDistanceToGoalHorizontalMeters(this.getDistanceToGoalVerticalMeters()));
    }
}
