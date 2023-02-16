package org.WaialuaRobotics359.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeM1Balance extends SequentialCommandGroup {

    public ConeM1Balance (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory ConeM1Balance = PathPlanner.loadPath("ConeM1Balance", new PathConstraints(3, 2));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(ConeM1Balance)
        ));
    }
}