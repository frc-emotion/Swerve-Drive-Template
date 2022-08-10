package frc.robot.commands.auton;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonSwerveController {

    public static Command executePath(Trajectory path){
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
            path,
            poseSupplier,
            kinematics,
            xController,
            yController,
            thetaController,
            outputModuleStates,
            requirements
    }
}
