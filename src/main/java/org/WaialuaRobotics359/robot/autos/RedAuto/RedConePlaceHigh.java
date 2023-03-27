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

public class RedConePlaceHigh extends SequentialCommandGroup {

    public RedConePlaceHigh (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        PathPlannerTrajectory RedConePlaceHigh= PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("ConePlaceHigh", new PathConstraints( .1 , .1)), Alliance.Red);
        Pose2d startpose = RedConePlaceHigh.getInitialHolonomicPose();

        //PathPlannerTrajectory ConeScoreHigh = PathPlanner.loadPath("ConeScoreHigh", new PathConstraints(.1, .1));

        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(RedConePlaceHigh)
        ));
    }
}