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

public class RedConeL1Tri extends SequentialCommandGroup {

    public RedConeL1Tri (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        PathPlannerTrajectory ConeL1Tri= PathPlannerTrajectory.transformTrajectoryForAlliance((PathPlanner.loadPath("ConeL12.5", new PathConstraints(5, 3.5))), Alliance.Red);
        PathPlannerTrajectory ConeL1TriTwo= PathPlannerTrajectory.transformTrajectoryForAlliance((PathPlanner.loadPath("ConeL12.5PartTwo", new PathConstraints(5, 3))), Alliance.Red);
        Pose2d startpose = ConeL1Tri.getInitialHolonomicPose();

        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(ConeL1Tri),
            autoBuilder.fullAuto(ConeL1TriTwo)
        ));
    }
}