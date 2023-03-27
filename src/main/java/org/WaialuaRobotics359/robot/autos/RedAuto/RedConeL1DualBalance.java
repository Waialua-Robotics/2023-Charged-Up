package org.WaialuaRobotics359.robot.autos.RedAuto;


import java.util.ArrayList;
import java.util.List;

import org.WaialuaRobotics359.robot.commands.swerve.PoseEstimator;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedConeL1DualBalance extends SequentialCommandGroup {

    public RedConeL1DualBalance (SwerveAutoBuilder autoBuilder, PoseEstimator s_poseEstimator) {

        List<PathPlannerTrajectory> RedConeL1DualBalance = new ArrayList<>();

        RedConeL1DualBalance.add(PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPathGroup("ConeL1DualBalance", new PathConstraints(5, 3), new PathConstraints(7,4)).get(0), Alliance.Red));
        RedConeL1DualBalance.add(PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPathGroup("ConeL1DualBalance", new PathConstraints(5, 3), new PathConstraints(7,4)).get(1), Alliance.Red));
        Pose2d startpose = RedConeL1DualBalance.get(0).getInitialHolonomicPose();
        //PathPlannerTrajectory RedConeL1DualBalance= PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("RedConeL1DualBalance", new PathConstraints(5, 3), new PathConstraints(7, 4)), Alliance.Red);

        //List<PathPlannerTrajectory> ConeL1DualBalance = PathPlanner.loadPathGroup("ConeL1DualBalance", new PathConstraints(5, 3), new PathConstraints(7, 4)); //3,2
        //PathPlannerTrajectory ConeL1DualBalance2 = PathPlanner.loadPath("ConeL1DualBalance2",new PathConstraints(3, 2));

        addCommands(new SequentialCommandGroup(
            //new InstantCommand(() -> s_LEDs.state = State.purple),
            new InstantCommand(()-> s_poseEstimator.resetPose(startpose)),
            autoBuilder.fullAuto(RedConeL1DualBalance)
            //autoBuilder.fullAuto(ConeL1DualBalance2)
        ));
    }
}