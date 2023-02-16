package org.WaialuaRobotics359.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeM3Balance extends SequentialCommandGroup {

    public ConeM3Balance (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory ConeM3Balance = PathPlanner.loadPath("ConeM3Balance", new PathConstraints(3, 2));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(ConeM3Balance)
        ));
    }
}