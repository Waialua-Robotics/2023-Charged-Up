package org.WaialuaRobotics359.robot.commands.autonomous;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.RobotContainer;
import org.WaialuaRobotics359.robot.subsystems.Intake;
import org.WaialuaRobotics359.robot.subsystems.Elevator;
import org.WaialuaRobotics359.robot.subsystems.Slide;
import org.WaialuaRobotics359.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ThrowCube extends CommandBase {
    private Wrist s_Wrist;
    private Intake s_Intake;

    private static int WristPosition;
    private static int StowPosition;

    public ThrowCube(Wrist s_Wrist, Intake s_Intake) {
        this.s_Wrist = s_Wrist;
        this.s_Intake = s_Intake;
        addRequirements(s_Wrist);
    }

    boolean finished = false; 

    private Timer Timer = new Timer();

    public void initialize(){
        WristPosition = Constants.Wrist.Cube.LowPosition;
        StowPosition = 1200;

        finished = false;
        Timer.reset();
        Timer.start();
    }

    @Override
    public void execute(){

        //s_Wrist.setDesiredPosition(WristPosition);
        //s_Wrist.goToPosition();

        s_Wrist.SetPrecentOut(1);

        if(Timer.hasElapsed(.1)){
        s_Intake.outake(-1);
        }
        
        if (Timer.hasElapsed(.2)){
            s_Wrist.Stop();
            s_Wrist.setDesiredPosition(StowPosition);
            s_Wrist.goToPosition();
            finished =true; 
        }

    }
    
    public boolean isFinished(){
        return finished;
    }
}