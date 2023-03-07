package org.WaialuaRobotics359.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveBack extends SequentialCommandGroup {

    public DriveBack (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory DriveBack= PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("driveBack", new PathConstraints(1, 1)), Alliance.Red);

        //PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("driveBack", new PathConstraints(1, 1)), Alliance.Blue);
        //PathPlanner.loadPath ("twomLine", new PathConstraints(1, 1));

        //PathPlannerTrajectory DriveRToL= 
        //PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("2mLine", new PathConstraints(1, 1)), Alliance.Red);

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(DriveBack)
        ));
    }
}