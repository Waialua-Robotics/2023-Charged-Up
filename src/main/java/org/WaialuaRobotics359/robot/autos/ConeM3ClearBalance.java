package org.WaialuaRobotics359.robot.autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeM3ClearBalance extends SequentialCommandGroup {

    public ConeM3ClearBalance (SwerveAutoBuilder autoBuilder) {

        List<PathPlannerTrajectory> ConeM3ClearBalance = PathPlanner.loadPathGroup("ConeM3ClearBalance", new PathConstraints(4, 1.5), new PathConstraints (3, 2));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(ConeM3ClearBalance)
        ));
    }
}