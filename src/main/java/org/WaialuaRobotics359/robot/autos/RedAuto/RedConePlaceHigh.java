package org.WaialuaRobotics359.robot.autos.RedAuto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedConePlaceHigh extends SequentialCommandGroup {

    public RedConePlaceHigh (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory RedConePlaceHigh= PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("ConePlaceHigh", new PathConstraints( .1 , .1)), Alliance.Red);

        //PathPlannerTrajectory ConeScoreHigh = PathPlanner.loadPath("ConeScoreHigh", new PathConstraints(.1, .1));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(RedConePlaceHigh)
        ));
    }
}