package org.WaialuaRobotics359.robot.autos.RedAuto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedCubeM2Balance extends SequentialCommandGroup {

    public RedCubeM2Balance (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory RedCubeM2Balance = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("CubeM2Balance", new PathConstraints(3, 2)), Alliance.Red);

        //PathPlannerTrajectory CubeM2Balance = PathPlanner.loadPath("CubeM2Balance", new PathConstraints(3, 2));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(RedCubeM2Balance)
        ));
    }
}