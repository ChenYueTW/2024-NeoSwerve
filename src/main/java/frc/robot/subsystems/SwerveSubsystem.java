package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceId.Neo;
import frc.robot.lib.IDashboardProvider;
import frc.robot.DeviceId.Encoder;
import frc.robot.Constants.MotorReverse;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.EncoderOffset;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase implements IDashboardProvider {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final AHRS gyro;
    private final SwerveDriveOdometry odometry;

    public SwerveSubsystem() {
        this.registerDashboard();
        this.frontLeft = new SwerveModule(
            Neo.frontLeftDrive,
            Neo.frontLeftTurn,
            Encoder.frontLeft,
            MotorReverse.FRONT_LEFT_DRIVE,
            MotorReverse.FRONT_LEFT_TURN,
            EncoderOffset.FRONT_LEFT,
            "frontLeft"
        );
        this.frontRight = new SwerveModule(
            Neo.frontRightDrive,
            Neo.frontRightTurn,
            Encoder.frontRight,
            MotorReverse.FRONT_RIGHT_DRIVE,
            MotorReverse.FRONT_RIGHT_TURN,
            EncoderOffset.FRONT_RIGHT,
            "frontRight"
        );
        this.backLeft = new SwerveModule(
            Neo.backwardLeftDrive,
            Neo.backwardLeftTurn,
            Encoder.backwardLeft,
            MotorReverse.BACK_LEFT_DRIVE,
            MotorReverse.BACK_LEFT_TURN,
            EncoderOffset.BACK_LEFT,
            "backLeft"
        );
        this.backRight = new SwerveModule(
            Neo.backwardRightDrive,
            Neo.backwardRightTurn,
            Encoder.backwardRight,
            MotorReverse.BACK_RIGHT_DRIVE,
            MotorReverse.BACK_RIGHT_TURN,
            EncoderOffset.BACK_RIGHT,
            "backRight"
        );
        this.gyro = new AHRS(SPI.Port.kMXP);
        this.odometry = new SwerveDriveOdometry(
            Constants.swerveDriveKinematics, this.gyro.getRotation2d(), this.getModulePosition()
        );
        this.gyro.reset();
    }

    @Override
    public void periodic() {
        this.odometry.update(this.gyro.getRotation2d(), getModulePosition());
    }

    public void resetGyro() {
        this.gyro.reset();
    }

    public void driveSwerve(double xSpeed, double ySpeed, double rotation, boolean field) {
        SwerveModuleState[] state = Constants.swerveDriveKinematics.toSwerveModuleStates(field ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.gyro.getRotation2d()) :
            new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );
        this.setModuleState(state);
    }

    public void autoDriveSwerve(ChassisSpeeds relativeSpeed) {
        SwerveModuleState[] state = Constants.swerveDriveKinematics.toSwerveModuleStates(relativeSpeed);
        this.setModuleState(state);
    }

    public void setModuleState(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    public SwerveModuleState[] getModuleState() {
        return new SwerveModuleState[] {
            this.frontLeft.getState(),
            this.frontRight.getState(),
            this.backLeft.getState(),
            this.backRight.getState()
        };
    }

    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.swerveDriveKinematics.toChassisSpeeds(this.getModuleState());
    }

    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    public double[] getLinearAccel() {
        return new double[] {
            this.gyro.getWorldLinearAccelX(),
            this.gyro.getWorldLinearAccelY(),
            this.gyro.getWorldLinearAccelZ()
        };
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putBoolean("IsMoving", this.gyro.isMoving());
        SmartDashboard.putNumberArray("WorldLinearAccel", this.getLinearAccel());
    }
}