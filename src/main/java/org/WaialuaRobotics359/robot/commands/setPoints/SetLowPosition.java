package org.WaialuaRobotics359.robot.commands.setPoints;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.RobotContainer;
import org.WaialuaRobotics359.robot.subsystems.Elevator;
import org.WaialuaRobotics359.robot.subsystems.Slide;
import org.WaialuaRobotics359.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetLowPosition extends CommandBase {
    private Wrist s_Wrist;
    private Elevator s_Elevator;
    private Slide s_Slide;

    private static int ElevatorPosition;
    private static int SlidePosition;
    private static int WristPosition;

    public SetLowPosition(Wrist s_Wrist, Elevator s_Elevator, Slide s_Slide) {
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
            ElevatorPosition = Constants.Elevator.Cube.LowPosition;
            SlidePosition = Constants.Slide.Cube.LowPosition;
            WristPosition = 11570;
        }else{
            ElevatorPosition = Constants.Elevator.Cone.LowPosition;
            SlidePosition = Constants.Slide.Cone.LowPosition;
            WristPosition = Constants.Wrist.Cone.LowPosition;
        }

        finished = false;
        Timer.reset();
        Timer.start();
    }

    @Override
    public void execute(){

        if(!RobotContainer.isCube){
            s_Wrist.setDesiredPosition(WristPosition);
            s_Wrist.goToPosition();
            
            if (Timer.hasElapsed(0.2)){
                s_Slide.setDesiredPosition(SlidePosition);
                s_Slide.goToPosition();
            }

            if (Timer.hasElapsed(.5)){
                    s_Elevator.setDesiredPosition(ElevatorPosition);
                s_Elevator.goToPosition();
                finished =true; 
            }
        }else{
            s_Wrist.setDesiredPosition(WristPosition);
            s_Wrist.goToPosition();
            
            if (Timer.hasElapsed(0.2)){
                s_Slide.setDesiredPosition(SlidePosition);
                s_Slide.goToPosition();
            }

            if (Timer.hasElapsed(.5)){
                    s_Elevator.setDesiredPosition(ElevatorPosition);
                s_Elevator.goToPosition();
                finished =true; 
            }
            
            /* 
            s_Wrist.setDesiredPosition(Constants.Wrist.SafePosition);
            s_Wrist.goToPosition();
        
            if (Timer.hasElapsed(0.1)){
                s_Slide.setDesiredPosition(SlidePosition);
                s_Slide.goToPosition();
            }
    
            if (s_Slide.GetPosition() < 40000 || Timer.hasElapsed(.3)){
                s_Elevator.setDesiredPosition(ElevatorPosition);
                s_Elevator.goToPosition();
            }
    
            if (s_Elevator.GetPosition() > 20000 && Timer.hasElapsed(.4)|| Timer.hasElapsed(.8)){
                s_Wrist.setDesiredPosition(WristPosition);
                s_Wrist.goToPosition();
                finished = true;
            }*/
        }

    }
    
    public boolean isFinished(){
        return finished;
    }
}