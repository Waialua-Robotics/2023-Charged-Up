package org.WaialuaRobotics359.robot.autos.AutoCommandGroup;

import org.WaialuaRobotics359.robot.commands.autonomous.*;
import org.WaialuaRobotics359.robot.commands.setPoints.*;
import org.WaialuaRobotics359.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeIntakeStow extends SequentialCommandGroup {

    public ConeIntakeStow (Wrist s_Wrist,Elevator s_Elevator, Slide s_Slide, Intake s_Intake) {


        addCommands(new SequentialCommandGroup(
            new AutoIntakeCone(s_Intake),
            new AutoWait(.2),
            new SetStowPosition(s_Wrist, s_Elevator, s_Slide)
        ));
    }
}