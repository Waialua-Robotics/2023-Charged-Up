package org.WaialuaRobotics359.robot.autos.RedAuto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedConeL1Auto extends SequentialCommandGroup {

    public RedConeL1Auto (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory RedConeL1Auto= PathPlannerTrajectory.transformTrajectoryForAlliance((PathPlanner.loadPath("ConeL1Auto", new PathConstraints(3, 2))), Alliance.Red);


        //PathPlannerTrajectory ConeL1Auto = PathPlanner.loadPath("ConeL1Auto", new PathConstraints(3, 2));
        //PathPlannerTrajectory ConeL1AutoReturn = PathPlanner.loadPath("ConeL1AutoReturn",new PathConstraints(3, 2));

        addCommands(new SequentialCommandGroup(
            //new InstantCommand(() -> s_LEDs.state = State.purple),
            autoBuilder.fullAuto(RedConeL1Auto)
            //autoBuilder.fullAuto(ConeL1AutoReturn)
        ));
    }
}