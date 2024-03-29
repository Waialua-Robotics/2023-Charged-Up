package org.WaialuaRobotics359.robot.autos;

import org.WaialuaRobotics359.robot.commands.swerve.PoseEstimator;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeR3DualCube extends SequentialCommandGroup {

    public ConeR3DualCube (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        PathPlannerTrajectory PathOne = PathPlanner.loadPath("ConeR3DualCube", new PathConstraints(3.3, 2)); 
        //PathPlannerTrajectory PathTwo = PathPlanner.loadPath("ConeL1DualLinkOut", new PathConstraints(5, 3));
        Pose2d startpose = PathOne.getInitialHolonomicPose();

        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(PathOne)
            //autoBuilder.fullAuto(ConeL1DualLinkOut)
        ));
    }
}