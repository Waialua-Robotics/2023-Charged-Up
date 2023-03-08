package org.WaialuaRobotics359.robot.autos.RedAuto;

import java.util.ArrayList;
import java.util.List;

import org.WaialuaRobotics359.robot.commands.swerve.PoseEstimator;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedConeM3ClearBalance extends SequentialCommandGroup {

    public RedConeM3ClearBalance (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

         List<PathPlannerTrajectory> RedConeM3ClearBalance = new ArrayList<>();

        RedConeM3ClearBalance.add(PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPathGroup("ConeM3ClearBalance", new PathConstraints(4, 1.5), new PathConstraints(3,2)).get(0), Alliance.Red));
        RedConeM3ClearBalance.add(PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPathGroup("ConeM3ClearBalance", new PathConstraints(4, 1.5), new PathConstraints(3,2)).get(1), Alliance.Red));
        Pose2d startpose = RedConeM3ClearBalance.get(0).getInitialHolonomicPose();

        //List<PathPlannerTrajectory> ConeM3ClearBalance = PathPlanner.loadPathGroup("ConeM3ClearBalance", new PathConstraints(4, 1.5), new PathConstraints (3, 2));

        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(RedConeM3ClearBalance)
        ));
    }
}