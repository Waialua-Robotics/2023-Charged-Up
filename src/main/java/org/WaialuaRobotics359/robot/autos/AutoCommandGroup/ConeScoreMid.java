package org.WaialuaRobotics359.robot.autos.AutoCommandGroup;

import org.WaialuaRobotics359.robot.commands.autonomous.*;
import org.WaialuaRobotics359.robot.commands.setPoints.*;
import org.WaialuaRobotics359.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeScoreMid extends SequentialCommandGroup {

    public ConeScoreMid (Wrist s_Wrist,Elevator s_Elevator, Slide s_Slide, Intake s_Intake) {


        addCommands(new SequentialCommandGroup(
            new SetMidPosition(s_Wrist, s_Elevator, s_Slide),
            new AutoWait(1.5),
            new AutoOuttakeCone(s_Intake),
            new SetStowPosition(s_Wrist, s_Elevator, s_Slide)
        ));
    }
}