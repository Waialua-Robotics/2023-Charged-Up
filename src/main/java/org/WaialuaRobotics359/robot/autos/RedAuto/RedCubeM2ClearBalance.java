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

public class RedCubeM2ClearBalance extends SequentialCommandGroup {

    public RedCubeM2ClearBalance (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        //PathPlannerState.transformStateForAlliance(PathPlanner.loadPath("RedCubeM2ClearBalance", new PathConstraints(3, 2)), Alliance.Red);

        List<PathPlannerTrajectory> RedCubeM2ClearBalance = new ArrayList<>();

        RedCubeM2ClearBalance.add(PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPathGroup("CubeM2ClearBalance", new PathConstraints(4, 1.5), new PathConstraints(3,3)).get(0), Alliance.Red));
        RedCubeM2ClearBalance.add(PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPathGroup("CubeM2ClearBalance", new PathConstraints(4, 1.5), new PathConstraints(3,3)).get(1), Alliance.Red));
        Pose2d startpose = RedCubeM2ClearBalance.get(0).getInitialHolonomicPose();

        //PathPlannerTrajectory RedCubeM2ClearBalance = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPathGroup("RedCubeM2ClearBalance", new PathConstraints(1, 1), new PathConstraints(3,2)).get(0), Alliance.Red);
        //List<PathPlannerTrajectory> something = {RedCubeM2ClearBalance, RedCubeM2ClearBalance};

        //List<PathPlannerTrajectory> RedCubeM2ClearBalance = PathPlanner.loadPathGroup("RedCubeM2ClearBalance", new PathConstraints(4, 1.5), new PathConstraints (3, 2));
        //PathPlannerTrajectory.transformTrajectoryForAlliance
        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(RedCubeM2ClearBalance)
        ));
    }
}