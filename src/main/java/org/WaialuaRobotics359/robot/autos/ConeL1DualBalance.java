package org.WaialuaRobotics359.robot.autos;

import java.util.List;

import org.WaialuaRobotics359.robot.commands.swerve.PoseEstimator;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeL1DualBalance extends SequentialCommandGroup {

    public ConeL1DualBalance (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        List<PathPlannerTrajectory> ConeL1DualBalance = PathPlanner.loadPathGroup("ConeL1DualBalance", new PathConstraints(5, 3), new PathConstraints(7, 4)); //3,2
        Pose2d startpose = ConeL1DualBalance.get(0).getInitialHolonomicPose();
        //PathPlannerTrajectory ConeL1DualBalance2 = PathPlanner.loadPath("ConeL1DualBalance2",new PathConstraints(3, 2));

        addCommands(new SequentialCommandGroup(
            //new InstantCommand(() -> s_LEDs.state = State.purple),
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(ConeL1DualBalance)
            //autoBuilder.fullAuto(ConeL1DualBalance2)
        ));
    }
}