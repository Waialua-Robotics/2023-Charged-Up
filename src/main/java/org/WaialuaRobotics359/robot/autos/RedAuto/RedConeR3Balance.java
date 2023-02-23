package org.WaialuaRobotics359.robot.autos.RedAuto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedConeR3Balance extends SequentialCommandGroup {

    public RedConeR3Balance (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory RedConeR3Balance = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("ConeR3Balance", new PathConstraints(2, 1)), Alliance.Red);

        //PathPlannerTrajectory ConeR3Balance = PathPlanner.loadPath("ConeR3Balance", new PathConstraints(2, 1));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(RedConeR3Balance)
        ));
    }
}