package frc.robot.lib;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.SwerveConstants;

public class LazySpark extends CANSparkMax{
    public LazySpark(int motorPort, boolean reverse) {
        super(motorPort, MotorType.kBrushless);
        this.restoreFactoryDefaults();
        this.setInverted(reverse);
        this.setIdleMode(IdleMode.kBrake);
        this.setSmartCurrentLimit(SwerveConstants.MAX_VOLTAGE);
    }
}
