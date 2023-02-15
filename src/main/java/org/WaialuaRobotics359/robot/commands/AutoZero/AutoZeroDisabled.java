package org.WaialuaRobotics359.robot.commands.AutoZero;

import org.WaialuaRobotics359.robot.subsystems.Slide;
import org.WaialuaRobotics359.robot.subsystems.Elevator;
import org.WaialuaRobotics359.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoZeroDisabled extends CommandBase {

    private Wrist s_Wrist;
    private Elevator s_Elevator;
    private Slide s_Slide;

    public AutoZeroDisabled(Wrist s_Wrist, Elevator s_Elevator, Slide s_Slide) {
        this.s_Wrist = s_Wrist;
        this.s_Elevator = s_Elevator;
        this.s_Slide = s_Slide;
        addRequirements(s_Wrist);
        addRequirements(s_Elevator);
        addRequirements(s_Slide);
    }

    public void initialize(){
        s_Wrist.setDesiredPosition((int)s_Wrist.GetPosition());
        s_Elevator.setDesiredPosition(s_Elevator.GetPosition());
        s_Slide.setDesiredPosition(s_Slide.GetPosition());

    }
    
    public boolean isFinished(){
        return true;
    }

}