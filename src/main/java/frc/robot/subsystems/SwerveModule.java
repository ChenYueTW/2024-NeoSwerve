package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.IDashboardProvider;
import frc.robot.lib.LazySpark;

public class SwerveModule implements IDashboardProvider{
    private final LazySpark driveMotor;
    private final LazySpark turnMotor;

    private final RelativeEncoder driveEncoder;
    private final CANcoder turnEncoder;

    private final PIDController turnPidController;

    private final String motorName;
    private double driveOutput;
    private double turnOutput;

    private final double turningEncoderOffset;

    public SwerveModule(
        int driveMotorPort, int turnMotorPort, int turnEncoderPort,
        boolean driveMotorReverse, boolean turnMotorReverse,
        double turnEncoderOffset, String motorName
    ){
        this.registerDashboard();

        this.driveMotor = new LazySpark(driveMotorPort, driveMotorReverse);
        this.turnMotor = new LazySpark(turnMotorPort, turnMotorReverse);

        this.driveEncoder = this.driveMotor.getEncoder();
        this.turnEncoder = new CANcoder(turnEncoderPort);

        this.driveEncoder.setPositionConversionFactor(SwerveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
        this.driveEncoder.setVelocityConversionFactor(SwerveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR);

        this.turnPidController = new PIDController(0.01015, 0.0001, 0);
        this.turnPidController.enableContinuousInput(-180, 180);

        this.motorName = motorName;
        this.turningEncoderOffset = turnEncoderOffset;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.driveEncoder.getVelocity(),
            Rotation2d.fromDegrees(this.getTurningEncoderPosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveEncoder.getPosition(),
            Rotation2d.fromDegrees(this.getTurningEncoderPosition())
        );
    }

    public double getTurningEncoderPosition() {
        return Units.rotationsToDegrees(this.turnEncoder.getAbsolutePosition().getValue()) - this.turningEncoderOffset;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getState().angle);

        this.driveOutput = state.speedMetersPerSecond / SwerveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        this.turnOutput = this.turnPidController.calculate(this.getState().angle.getDegrees(), state.angle.getDegrees());

        this.driveMotor.set(this.driveOutput);
        this.turnMotor.set(this.turnOutput);
    }

    public void setAutoDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getState().angle);

        this.driveOutput = state.speedMetersPerSecond;
        this.turnOutput = this.turnPidController.calculate(this.getState().angle.getDegrees(), state.angle.getDegrees());

        this.driveMotor.set(this.driveOutput);
        this.turnMotor.set(this.turnOutput);
    }

    @Override
    public void putDashboard() {
        // SmartDashboard.putNumber(this.motorName + " DrivePosition", this.driveEncoder.getPosition());
        // SmartDashboard.putNumber(this.motorName + " DriveVelocity", this.driveEncoder.getVelocity());
        SmartDashboard.putNumber(this.motorName + " TurnPosition", this.getTurningEncoderPosition());
        SmartDashboard.putNumber(this.motorName + " TurnVelocity", this.turnEncoder.getVelocity().getValue());
        SmartDashboard.putNumber(this.motorName + " DriveMotor", this.driveOutput);
        SmartDashboard.putNumber(this.motorName + " TurnMotor", this.turnOutput);
    }

    public void stop() {
        this.driveMotor.set(0);
        this.turnMotor.set(0);
    }
}
