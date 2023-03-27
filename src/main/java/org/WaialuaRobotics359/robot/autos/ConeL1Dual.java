package org.WaialuaRobotics359.robot.autos;

import org.WaialuaRobotics359.robot.commands.swerve.PoseEstimator;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeL1Dual extends SequentialCommandGroup {

    public ConeL1Dual (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        PathPlannerTrajectory ConeL1Dual = PathPlanner.loadPath("ConeL1Dual", new PathConstraints(3, 2));
        Pose2d startpose = ConeL1Dual.getInitialHolonomicPose();
        //PathPlannerTrajectory ConeL1DualReturn = PathPlanner.loadPath("ConeL1DualReturn",new PathConstraints(3, 2));

        addCommands(new SequentialCommandGroup(
            //new InstantCommand(() -> s_LEDs.state = State.purple),
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(ConeL1Dual)
            //autoBuilder.fullAuto(ConeL1AutoReturn)
        ));
    }
}