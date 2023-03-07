package org.WaialuaRobotics359.robot.autos.RedAuto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedCubeM2ClearBalance extends SequentialCommandGroup {

    public RedCubeM2ClearBalance (SwerveAutoBuilder autoBuilder) {

        //PathPlannerState.transformStateForAlliance(PathPlanner.loadPath("RedCubeM2ClearBalance", new PathConstraints(3, 2)), Alliance.Red);

        List<PathPlannerTrajectory> RedCubeM2ClearBalance = new ArrayList<>();

        RedCubeM2ClearBalance.add(PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPathGroup("CubeM2ClearBalance", new PathConstraints(4, 1.5), new PathConstraints(3,2)).get(0), Alliance.Red));
        RedCubeM2ClearBalance.add(PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPathGroup("CubeM2ClearBalance", new PathConstraints(4, 1.5), new PathConstraints(3,2)).get(1), Alliance.Red));

        //PathPlannerTrajectory RedCubeM2ClearBalance = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPathGroup("RedCubeM2ClearBalance", new PathConstraints(1, 1), new PathConstraints(3,2)).get(0), Alliance.Red);
        //List<PathPlannerTrajectory> something = {RedCubeM2ClearBalance, RedCubeM2ClearBalance};

        //List<PathPlannerTrajectory> RedCubeM2ClearBalance = PathPlanner.loadPathGroup("RedCubeM2ClearBalance", new PathConstraints(4, 1.5), new PathConstraints (3, 2));
        //PathPlannerTrajectory.transformTrajectoryForAlliance
        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(RedCubeM2ClearBalance)
        ));
    }
}