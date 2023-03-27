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

public class ConeM3ClearBalance extends SequentialCommandGroup {

    public ConeM3ClearBalance (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        List<PathPlannerTrajectory> ConeM3ClearBalance = PathPlanner.loadPathGroup("ConeM3ClearBalance", new PathConstraints(4, 1.5), new PathConstraints (3, 2));
        Pose2d startpose = ConeM3ClearBalance.get(0).getInitialHolonomicPose();

        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(ConeM3ClearBalance)
        ));
    }
}