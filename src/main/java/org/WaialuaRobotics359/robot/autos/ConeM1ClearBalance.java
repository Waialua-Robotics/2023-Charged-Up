package org.WaialuaRobotics359.robot.autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeM1ClearBalance extends SequentialCommandGroup {

    public ConeM1ClearBalance (SwerveAutoBuilder autoBuilder) {

        List<PathPlannerTrajectory> ConeM1ClearBalance = PathPlanner.loadPathGroup("ConeM1ClearBalance", new PathConstraints(4, 1.5), new PathConstraints (3, 2));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(ConeM1ClearBalance)
        ));
    }
}