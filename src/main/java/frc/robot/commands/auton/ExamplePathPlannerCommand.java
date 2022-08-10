package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.SwerveSubsytem;

public class ExamplePathPlannerCommand extends ParallelCommandGroup {
    
    public ExamplePathPlannerCommand(SwerveSubsytem swerveSubsytem, PathPlannerTrajectory path){

        addCommands(SwerveController.executePath(swerveSubsytem, path)
        );
    }
}
