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

public class RedConeR3DualBalance extends SequentialCommandGroup {

    public RedConeR3DualBalance (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        PathPlannerTrajectory RedConeR3DualBalance = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("ConeR3DualBalance", new PathConstraints(2, 1)), Alliance.Red);
        Pose2d startpose = RedConeR3DualBalance.getInitialHolonomicPose();

      //  PathPlannerTrajectory ConeR3DualBalance = PathPlanner.loadPath("ConeR3DualBalance", new PathConstraints(2, 1));

        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(RedConeR3DualBalance)
        ));
    }
}