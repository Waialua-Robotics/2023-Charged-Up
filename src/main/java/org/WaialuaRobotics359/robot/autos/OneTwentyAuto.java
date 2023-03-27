package org.WaialuaRobotics359.robot.autos;

import org.WaialuaRobotics359.robot.commands.swerve.PoseEstimator;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneTwentyAuto extends SequentialCommandGroup {

    public OneTwentyAuto (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        PathPlannerTrajectory OneTwentyAuto = PathPlanner.loadPath("OneTwenty", new PathConstraints(3, 2));
        Pose2d startpose = OneTwentyAuto.getInitialHolonomicPose();


        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(OneTwentyAuto)
        ));
    }
}