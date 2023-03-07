package org.WaialuaRobotics359.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeL1Dual extends SequentialCommandGroup {

    public ConeL1Dual (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory ConeL1Dual = PathPlanner.loadPath("ConeL1Dual", new PathConstraints(3, 2));
        //PathPlannerTrajectory ConeL1DualReturn = PathPlanner.loadPath("ConeL1DualReturn",new PathConstraints(3, 2));

        addCommands(new SequentialCommandGroup(
            //new InstantCommand(() -> s_LEDs.state = State.purple),
            autoBuilder.fullAuto(ConeL1Dual)
            //autoBuilder.fullAuto(ConeL1AutoReturn)
        ));
    }
}