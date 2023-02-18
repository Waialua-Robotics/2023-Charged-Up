package org.WaialuaRobotics359.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeR3Dual extends SequentialCommandGroup {

    public ConeR3Dual (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory ConeR3Dual = PathPlanner.loadPath("ConeR3Dual", new PathConstraints(3, 1.5));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(ConeR3Dual)
        ));
    }
}