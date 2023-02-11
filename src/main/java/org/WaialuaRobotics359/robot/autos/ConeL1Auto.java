package org.WaialuaRobotics359.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeL1Auto extends SequentialCommandGroup {

    public ConeL1Auto (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory ConeL1Auto = PathPlanner.loadPath("ConeL1Auto", new PathConstraints(1, 1));
        PathPlannerTrajectory ConeL1AutoReturn = PathPlanner.loadPath("ConeL1AutoReturn",new PathConstraints(1, 1));

        addCommands(new SequentialCommandGroup(
            //new InstantCommand(() -> s_LEDs.state = State.purple),
            autoBuilder.fullAuto(ConeL1Auto),
            autoBuilder.fullAuto(ConeL1AutoReturn)
        ));
    }
}