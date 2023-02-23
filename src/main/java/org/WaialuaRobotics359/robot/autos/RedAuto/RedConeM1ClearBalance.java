package org.WaialuaRobotics359.robot.autos.RedAuto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedConeM1ClearBalance extends SequentialCommandGroup {

    public RedConeM1ClearBalance (SwerveAutoBuilder autoBuilder) {

          List<PathPlannerTrajectory> RedConeM1ClearBalance = new ArrayList<>();

        RedConeM1ClearBalance.add(PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPathGroup("ConeM1ClearBalance", new PathConstraints(4, 1.5), new PathConstraints(3,2)).get(0), Alliance.Red));
        RedConeM1ClearBalance.add(PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPathGroup("ConeM1ClearBalance", new PathConstraints(4, 1.5), new PathConstraints(3,2)).get(1), Alliance.Red));

        //List<PathPlannerTrajectory> ConeM1ClearBalance = PathPlanner.loadPathGroup("ConeM1ClearBalance", new PathConstraints(4, 1.5), new PathConstraints (3, 2));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(RedConeM1ClearBalance)
        ));
    }
}