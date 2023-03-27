package org.WaialuaRobotics359.robot.autos.RedAuto;

import org.WaialuaRobotics359.robot.commands.swerve.PoseEstimator;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedCubeM2Balance extends SequentialCommandGroup {

    public RedCubeM2Balance (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        PathPlannerTrajectory RedCubeM2Balance = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("CubeM2Balance", new PathConstraints(3, 2)), Alliance.Red);
        Pose2d startpose = RedCubeM2Balance.getInitialHolonomicPose();

        //PathPlannerTrajectory CubeM2Balance = PathPlanner.loadPath("CubeM2Balance", new PathConstraints(3, 2));

        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(RedCubeM2Balance)
        ));
    }
}