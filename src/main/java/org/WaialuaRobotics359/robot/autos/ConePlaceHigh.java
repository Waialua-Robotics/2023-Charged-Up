package org.WaialuaRobotics359.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConePlaceHigh extends SequentialCommandGroup {

    public ConePlaceHigh (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory ConeScoreHigh = PathPlanner.loadPath("ConeScoreHigh", new PathConstraints(.1, .1));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(ConeScoreHigh)
        ));
    }
}