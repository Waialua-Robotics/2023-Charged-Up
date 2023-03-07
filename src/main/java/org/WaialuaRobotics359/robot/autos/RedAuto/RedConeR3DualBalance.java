package org.WaialuaRobotics359.robot.autos.RedAuto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedConeR3DualBalance extends SequentialCommandGroup {

    public RedConeR3DualBalance (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory RedConeR3DualBalance = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("ConeR3DualBalance", new PathConstraints(2, 1)), Alliance.Red);

      //  PathPlannerTrajectory ConeR3DualBalance = PathPlanner.loadPath("ConeR3DualBalance", new PathConstraints(2, 1));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(RedConeR3DualBalance)
        ));
    }
}