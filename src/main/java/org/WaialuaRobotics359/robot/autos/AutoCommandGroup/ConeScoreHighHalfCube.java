package org.WaialuaRobotics359.robot.autos.AutoCommandGroup;

import org.WaialuaRobotics359.robot.RobotContainer;
import org.WaialuaRobotics359.robot.commands.autonomous.*;
import org.WaialuaRobotics359.robot.commands.setPoints.*;
import org.WaialuaRobotics359.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeScoreHighHalfCube extends SequentialCommandGroup {

    public ConeScoreHighHalfCube (Wrist s_Wrist,Elevator s_Elevator, Slide s_Slide, Intake s_Intake) {


        addCommands(new SequentialCommandGroup(
            new SetHighPosition(s_Wrist, s_Elevator, s_Slide),
            new AutoWait(1.3),
            new AutoOuttakeCone(s_Intake),
            new InstantCommand(() -> RobotContainer.isCube = true),
            new SetStandPosition(s_Wrist, s_Elevator, s_Slide)
            //new AutoWait(.1)  new InstantCommand(() -> isCube = true)
        ));
    }
}