package frc.robot.Auto;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class PathPlannerCmd extends SequentialCommandGroup{
    private final SwerveSubsystem swerveSubsystem;
    private final PIDConstants translationPid;
    private final PIDConstants rotationPid;
    private final PathPlannerPath path;

    public PathPlannerCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.translationPid = new PIDConstants(4.0, 0, 0);
        this.rotationPid = new PIDConstants(3.0, 0, 0);
        this.path = PathPlannerPath.fromPathFile("SwervePath");

        addCommands(this.autoDrive());
    }

    public Command autoDrive() {
        return new FollowPathHolonomic(
            this.path,
            this.swerveSubsystem::getPose,
            this.swerveSubsystem::getSpeeds,
            this.swerveSubsystem::autoDriveSwerve,
            new HolonomicPathFollowerConfig(
                this.translationPid,
                this.rotationPid,
                SwerveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND,
                SwerveConstants.TRACK_WIDTH / 2,
                new ReplanningConfig()
            ),
            () -> {
                var allience = DriverStation.getAlliance();
                if (allience.isPresent()) {
                    return allience.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this.swerveSubsystem
        ); 
    }
}
