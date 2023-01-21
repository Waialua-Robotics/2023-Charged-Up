package org.WaialuaRobotics359.robot.commands.setPoints;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Elevator;
import org.WaialuaRobotics359.robot.subsystems.Slide;
import org.WaialuaRobotics359.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetLowPosition extends CommandBase {
    private Wrist s_Wrist;
    private Elevator s_Elevator;
    private Slide s_Slide;

    public SetLowPosition(Wrist s_Wrist, Elevator s_Elevator, Slide s_Slide) {
        this.s_Wrist = s_Wrist;
        this.s_Elevator = s_Elevator;
        this.s_Slide = s_Slide;
        addRequirements(s_Wrist);
        addRequirements(s_Elevator);
        addRequirements(s_Slide);
    }

    public void initialize(){

    }

    @Override
    public void execute(){
    
        s_Wrist.setDesiredPosition(Constants.Wrist.LowPosition);
        s_Wrist.goToPosition();

        s_Elevator.setDesiredPosition(Constants.Elevator.LowPosition);
        s_Elevator.goToPosition();

        s_Slide.setDesiredPosition(Constants.Slide.LowPosition);
        s_Slide.goToPosition();

    }
    
    public boolean isFinished(){
        return true;
    }
}