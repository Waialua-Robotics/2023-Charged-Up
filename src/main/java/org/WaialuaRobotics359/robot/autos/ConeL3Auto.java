package org.WaialuaRobotics359.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeL3Auto extends SequentialCommandGroup {

    public ConeL3Auto (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory ConeL3Auto = PathPlanner.loadPath("ConeL3Auto", new PathConstraints(3, 2));
        PathPlannerTrajectory ConeL3AutoReturn = PathPlanner.loadPath("ConeL3AutoReturn",new PathConstraints(2, 2));

        addCommands(new SequentialCommandGroup(
            //new InstantCommand(() -> s_LEDs.state = State.purple),
            autoBuilder.fullAuto(ConeL3Auto),
            autoBuilder.fullAuto(ConeL3AutoReturn)
        ));
    }
}