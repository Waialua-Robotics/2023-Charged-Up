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

public class RedDriveBack extends SequentialCommandGroup {

    public RedDriveBack (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        PathPlannerTrajectory RedDriveBack= PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("driveBack", new PathConstraints(1, 1)), Alliance.Blue);
        Pose2d startpose = RedDriveBack.getInitialHolonomicPose();

        //PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("driveBack", new PathConstraints(1, 1)), Alliance.Blue);
        //PathPlanner.loadPath ("twomLine", new PathConstraints(1, 1));

        //PathPlannerTrajectory DriveRToL= 
        //PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("2mLine", new PathConstraints(1, 1)), Alliance.Red);

        addCommands(new SequentialCommandGroup(
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(RedDriveBack)
        ));
    }
}