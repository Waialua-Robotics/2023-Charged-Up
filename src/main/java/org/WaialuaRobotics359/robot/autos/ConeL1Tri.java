package org.WaialuaRobotics359.robot.autos;

import org.WaialuaRobotics359.robot.commands.swerve.PoseEstimator;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeL1Tri extends SequentialCommandGroup {

    public ConeL1Tri (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        PathPlannerTrajectory ConeL1Tri = PathPlanner.loadPath("ConeL12.5", new PathConstraints(5, 2.5));
        PathPlannerTrajectory ConeL1TriTwo = PathPlanner.loadPath("ConeL12.5PartTwo", new PathConstraints(5, 3));
        Pose2d startpose = ConeL1Tri.getInitialHolonomicPose();

        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(ConeL1Tri),
            autoBuilder.fullAuto(ConeL1TriTwo)
        ));
    }
}