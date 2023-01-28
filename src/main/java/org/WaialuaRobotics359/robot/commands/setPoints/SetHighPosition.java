package org.WaialuaRobotics359.robot.commands.setPoints;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Elevator;
import org.WaialuaRobotics359.robot.subsystems.Slide;
import org.WaialuaRobotics359.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetHighPosition extends CommandBase {
    private Wrist s_Wrist;
    private Elevator s_Elevator;
    private Slide s_Slide;

    public SetHighPosition(Wrist s_Wrist, Elevator s_Elevator, Slide s_Slide) {
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
    
        s_Wrist.setDesiredPosition(Constants.Wrist.SafePosition);
        s_Wrist.goToPosition();

        Timer.delay(.5);

        s_Elevator.setDesiredPosition(Constants.Elevator.HighPosition);
        s_Elevator.goToPosition();

        Timer.delay(.5);

        s_Slide.setDesiredPosition(Constants.Slide.HighPosition);
        s_Slide.goToPosition();

        s_Wrist.setDesiredPosition(Constants.Wrist.HighPosition);
        s_Wrist.goToPosition();

    }
    
    public boolean isFinished(){
        return true;
    }
}
    

