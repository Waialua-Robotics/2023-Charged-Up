package org.WaialuaRobotics359.robot.autos.RedAuto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedConeR3Dual extends SequentialCommandGroup {

    public RedConeR3Dual (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory RedConeR3Dual = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("ConeR3Dual", new PathConstraints(3, 1.5)), Alliance.Red);

       // PathPlannerTrajectory ConeR3Dual = PathPlanner.loadPath("ConeR3Dual", new PathConstraints(3, 1.5));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(RedConeR3Dual)
        ));
    }
}