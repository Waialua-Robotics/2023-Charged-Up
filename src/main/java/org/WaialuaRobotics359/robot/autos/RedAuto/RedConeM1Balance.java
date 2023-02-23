package org.WaialuaRobotics359.robot.autos.RedAuto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedConeM1Balance extends SequentialCommandGroup {

    public RedConeM1Balance (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory RedConeM1Balance= PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("ConeM1Balance", new PathConstraints(3, 2)), Alliance.Red);

        //PathPlannerTrajectory ConeM1Balance = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("ConeM1Balance", new PathConstraints(3, 2)), Alliance.Red);

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(RedConeM1Balance)
        ));
    }
}