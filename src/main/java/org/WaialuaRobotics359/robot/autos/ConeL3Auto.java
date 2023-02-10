package org.WaialuaRobotics359.robot.autos;

import org.WaialuaRobotics359.robot.Constants;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeL3Auto extends SequentialCommandGroup {

    public ConeL3Auto (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory ConeL3Auto = PathPlanner.loadPath("ConeL3Auto", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        addCommands(new SequentialCommandGroup(
            //new InstantCommand(() -> s_LEDs.state = State.purple),
            autoBuilder.fullAuto(ConeL3Auto)
        ));
    }
}