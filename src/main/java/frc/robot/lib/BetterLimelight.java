package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimelightCamera;
import frc.robot.Constants.LimelightConstants;

public class BetterLimelight {
    private final NetworkTable table;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry tid;

    public BetterLimelight(String key) {
        this.table = NetworkTableInstance.getDefault().getTable(key);
        this.tx = this.table.getEntry("tx");
        this.ty = this.table.getEntry("ty");
        this.tid = this.table.getEntry("tid");
        this.table.getEntry("ledMode").setNumber(LimelightCamera.ledMode);
        this.table.getEntry("camMode").setNumber(LimelightCamera.camMode);
        this.table.getEntry("crop").setDoubleArray(LimelightCamera.cameraPose);
    }

    public double getDistanceToGoalVerticalMeters() {
        double verticalOffset = this.ty.getDouble(0.0);

        double mountAngleDeg = LimelightConstants.MOUNT_ANGLE_DEG;
        double lensHeightMeters = LimelightConstants.LENS_HEIGHT_METERS;
        double goalHeightMeters = LimelightConstants.GOAL_HEIGHT_METERS;

        double angleToGoalDeg = mountAngleDeg + verticalOffset;
        double angleToGoalRad = angleToGoalDeg * (Math.PI / 180.0);

        return Math.abs((goalHeightMeters - lensHeightMeters) / Math.tan(angleToGoalRad));
    }

    public double getDistanceToGoalHorizontalMeters(double distanceToGoalVerticalMeters) {
        if (distanceToGoalVerticalMeters == -1) {
            distanceToGoalVerticalMeters = this.getDistanceToGoalVerticalMeters();
        }
        double horizontalOffset = this.tx.getDouble(0.0);

        double horizontalOffsetRad = horizontalOffset * (Math.PI / 180.0);

        return (Math.tan(horizontalOffsetRad) * distanceToGoalVerticalMeters) - LimelightConstants.HORIZONTAL_OFFSET_METERS;
    }

    public double getAprilTagId() {
        return this.tid.getDouble(0.0);
    }
}
