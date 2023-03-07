package org.WaialuaRobotics359.robot.autos.RedAuto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedConeL3Auto extends SequentialCommandGroup {

    public RedConeL3Auto (SwerveAutoBuilder autoBuilder) {


        PathPlannerTrajectory RedConeL3Auto= PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("ConeL3Auto", new PathConstraints(3, 2)), Alliance.Red);
        PathPlannerTrajectory RedConeL3AutoReturn= PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("ConeL3AutoReturn", new PathConstraints(4, 2)), Alliance.Red);
       
       // PathPlannerTrajectory ConeL3Auto = PathPlanner.loadPath("ConeL3Auto", new PathConstraints(3, 2));
       // PathPlannerTrajectory ConeL3AutoReturn = PathPlanner.loadPath("ConeL3AutoReturn",new PathConstraints(4, 2));

        addCommands(new SequentialCommandGroup(
            //new InstantCommand(() -> s_LEDs.state = State.purple),
            autoBuilder.fullAuto(RedConeL3Auto),
            autoBuilder.fullAuto(RedConeL3AutoReturn)
        ));
    }
}