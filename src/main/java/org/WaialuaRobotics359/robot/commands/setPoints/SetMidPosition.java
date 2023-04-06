package org.WaialuaRobotics359.robot.commands.setPoints;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.RobotContainer;
import org.WaialuaRobotics359.robot.subsystems.Elevator;
import org.WaialuaRobotics359.robot.subsystems.Slide;
import org.WaialuaRobotics359.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetMidPosition extends CommandBase {
    private Wrist s_Wrist;
    private Elevator s_Elevator;
    private Slide s_Slide;

    private static int ElevatorPosition;
    private static int SlidePosition;
    private static int WristPosition;

    public SetMidPosition(Wrist s_Wrist, Elevator s_Elevator, Slide s_Slide) {
        this.s_Wrist = s_Wrist;
        this.s_Elevator = s_Elevator;
        this.s_Slide = s_Slide;
        addRequirements(s_Wrist);
        addRequirements(s_Elevator);
        addRequirements(s_Slide);
    }

    boolean finished = false; 

    private Timer Timer = new Timer();

    public void initialize(){
        if (RobotContainer.isCube){
            ElevatorPosition = Constants.Elevator.Cube.MidPosition;
            SlidePosition = Constants.Slide.Cube.MidPosition;
            WristPosition = Constants.Wrist.Cube.MidPosition;
        }else{
            ElevatorPosition = Constants.Elevator.Cone.MidPosition;
            SlidePosition = Constants.Slide.Cone.MidPosition;
            WristPosition = Constants.Wrist.Cone.MidPosition;
        }

        finished = false;
        
        Timer.reset();

        Timer.start();
    }

    @Override
    public void execute(){

        s_Wrist.setDesiredPosition(Constants.Wrist.SafePosition);
        s_Wrist.goToPosition();
        if (s_Wrist.inSafe()){
            s_Elevator.setDesiredPosition(ElevatorPosition);
            s_Elevator.goToPosition();
        }
        if (Timer.hasElapsed(.6)){
            s_Slide.setDesiredPosition(SlidePosition);
            s_Slide.goToPosition();
        }
        if (Timer.hasElapsed(.8)){
            s_Wrist.setDesiredPosition(WristPosition);
            s_Wrist.goToPosition();
            finished = true;
        }
    

    }
    
    public boolean isFinished(){
        return finished;
    }
}