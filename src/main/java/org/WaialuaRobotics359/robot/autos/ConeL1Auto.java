package org.WaialuaRobotics359.robot.autos;

import org.WaialuaRobotics359.robot.commands.swerve.PoseEstimator;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeL1Auto extends SequentialCommandGroup {

    public ConeL1Auto (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        PathPlannerTrajectory ConeL1Auto = PathPlanner.loadPath("ConeL1Auto", new PathConstraints(3, 2));
        Pose2d startpose = ConeL1Auto.getInitialHolonomicPose();
        //PathPlannerTrajectory ConeL1AutoReturn = PathPlanner.loadPath("ConeL1AutoReturn",new PathConstraints(3, 2));

        addCommands(new SequentialCommandGroup(
            //new InstantCommand(() -> s_LEDs.state = State.purple),
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(ConeL1Auto)
            //autoBuilder.fullAuto(ConeL1AutoReturn)
        ));
    }
}