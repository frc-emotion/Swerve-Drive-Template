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
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsytem;

/**
 * Static method that starts swerve controller command for ease of use
 * TODO: Add path planner event markers
 */
public final class SwerveController {

  public static Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, SwerveSubsytem swerveSubsystem) {

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
            swerveSubsystem.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         new PPSwerveControllerCommand(
             traj, 
             swerveSubsystem::getPose, // Pose supplier
             DriveConstants.kDriveKinematics, // SwerveDriveKinematics
             xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             yController, // Y controller (usually the same values as X controller)
             thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             swerveSubsystem::setModuleStates, // Module states consumer
             swerveSubsystem // Requires this drive subsystem
         )
          
         //new InstantCommand(() -> swerveSubsystem.stopModules())
     ); 
  }
}
