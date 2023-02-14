package org.WaialuaRobotics359.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeL1DualBalance extends SequentialCommandGroup {

    public ConeL1DualBalance (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory ConeL1DualBalance = PathPlanner.loadPath("ConeL1DualBalance", new PathConstraints(5, 3)); //3,2
        PathPlannerTrajectory ConeL1DualBalance2 = PathPlanner.loadPath("ConeL1DualBalance2",new PathConstraints(3, 2));

        addCommands(new SequentialCommandGroup(
            //new InstantCommand(() -> s_LEDs.state = State.purple),
            autoBuilder.fullAuto(ConeL1DualBalance),
            autoBuilder.fullAuto(ConeL1DualBalance2)
        ));
    }
}