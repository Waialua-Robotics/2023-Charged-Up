package org.WaialuaRobotics359.robot.autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CubeM2ClearBalance extends SequentialCommandGroup {

    public CubeM2ClearBalance (SwerveAutoBuilder autoBuilder) {

        List<PathPlannerTrajectory> CubeM2ClearBalance = PathPlanner.loadPathGroup("CubeM2ClearBalance", new PathConstraints(4, 1.5), new PathConstraints (3, 2));
        //PathPlannerTrajectory.transformTrajectoryForAlliance
        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(CubeM2ClearBalance)
        ));
    }
}