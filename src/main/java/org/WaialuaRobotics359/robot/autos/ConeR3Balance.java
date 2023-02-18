package org.WaialuaRobotics359.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeR3Balance extends SequentialCommandGroup {

    public ConeR3Balance (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory ConeR3Balance = PathPlanner.loadPath("ConeR3Balance", new PathConstraints(2, 1));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(ConeR3Balance)
        ));
    }
}