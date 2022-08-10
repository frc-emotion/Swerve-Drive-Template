package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsytem;

public final class SwerveController {

    public static Command executePath(SwerveSubsytem swerveSubsytem, PathPlannerTrajectory path){
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
            path,
            swerveSubsytem::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerveSubsytem::setModuleStates,
            swerveSubsytem);
    
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsytem.resetOdometry(path.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> swerveSubsytem.stopModules()));
    }
}
