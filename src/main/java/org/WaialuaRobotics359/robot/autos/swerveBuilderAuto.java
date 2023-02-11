package org.WaialuaRobotics359.robot.autos;

import org.WaialuaRobotics359.robot.Constants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class swerveBuilderAuto extends SequentialCommandGroup {

    public swerveBuilderAuto (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory ComplexAuto = PathPlanner.loadPath("ComplexAuto", new PathConstraints(1, 1));
        PathPlannerTrajectory ComplexAuto2 = PathPlanner.loadPath("ComplexAuto2", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        addCommands(new SequentialCommandGroup(
            //new InstantCommand(() -> s_LEDs.state = State.purple),
            autoBuilder.fullAuto(ComplexAuto),
            autoBuilder.fullAuto(ComplexAuto2)
        ));
    }
}