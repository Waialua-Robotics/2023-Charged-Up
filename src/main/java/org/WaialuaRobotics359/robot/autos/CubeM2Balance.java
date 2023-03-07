package org.WaialuaRobotics359.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CubeM2Balance extends SequentialCommandGroup {

    public CubeM2Balance (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory CubeM2Balance = PathPlanner.loadPath("CubeM2Balance", new PathConstraints(3, 2));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(CubeM2Balance)
        ));
    }
}