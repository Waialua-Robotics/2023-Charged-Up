package org.WaialuaRobotics359.robot.autos;

import java.util.List;

import org.WaialuaRobotics359.robot.commands.swerve.PoseEstimator;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CubeM2ClearBalancePlusOne extends SequentialCommandGroup {

    public CubeM2ClearBalancePlusOne (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        List<PathPlannerTrajectory> CubeM2ClearBalance = PathPlanner.loadPathGroup("CubeM2ClearBalancePlusOne", new PathConstraints(4, 1.5), new PathConstraints (3, 2));
        Pose2d startpose = CubeM2ClearBalance.get(0).getInitialHolonomicPose();
        //PathPlannerTrajectory.transformTrajectoryForAlliance
        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(CubeM2ClearBalance)
        ));
    }
}