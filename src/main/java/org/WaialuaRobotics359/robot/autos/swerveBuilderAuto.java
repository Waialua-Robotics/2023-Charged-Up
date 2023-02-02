package org.WaialuaRobotics359.robot.autos;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class swerveBuilderAuto extends SequentialCommandGroup {

    public swerveBuilderAuto (SwerveAutoBuilder autoBuilder) {

        PathPlannerTrajectory ComplexAuto = PathPlanner.loadPath("ComplexAuto", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        addCommands(new SequentialCommandGroup(
            autoBuilder.fullAuto(ComplexAuto)
        ));
    }
}