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

public class RedConeM1Balance extends SequentialCommandGroup {

    public RedConeM1Balance (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        PathPlannerTrajectory RedConeM1Balance= PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("ConeM1Balance", new PathConstraints(3, 2)), Alliance.Red);
        Pose2d startpose = RedConeM1Balance.getInitialHolonomicPose();

        //PathPlannerTrajectory ConeM1Balance = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("ConeM1Balance", new PathConstraints(3, 2)), Alliance.Red);

        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(RedConeM1Balance)
        ));
    }
}