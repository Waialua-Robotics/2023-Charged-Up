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

public class RedConeR3Dual extends SequentialCommandGroup {

    public RedConeR3Dual (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        PathPlannerTrajectory RedConeR3Dual = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("ConeR3Dual", new PathConstraints(3, 1.5)), Alliance.Red);
        Pose2d startpose = RedConeR3Dual.getInitialHolonomicPose();

       // PathPlannerTrajectory ConeR3Dual = PathPlanner.loadPath("ConeR3Dual", new PathConstraints(3, 1.5));

        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(RedConeR3Dual)
        ));
    }
}