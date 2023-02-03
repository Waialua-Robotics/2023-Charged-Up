package org.WaialuaRobotics359.robot.commands;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Slide;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoZeroSlide extends CommandBase {
    private Slide s_Slide;
    
    private double ZeroCurrent = 1;
    private boolean finished =  false; 

    public AutoZeroSlide(Slide s_Slide) {
        this.s_Slide = s_Slide;
        addRequirements(s_Slide);
    }

    public void initialize() {

        s_Slide.SetPosition(Constants.Slide.forwardSoftLimit +1000);

    }

    @Override
    public void execute() {

        //Auto Zero
        if (s_Slide.getCurrent() > ZeroCurrent){
            s_Slide.Stop();
            s_Slide.SetHomePosition();
            finished =  true; 
        }else{
            s_Slide.SetPrecentOut(-.01);
        }

    }
    
    @Override
    public boolean isFinished() {
        return finished;
    }

}
