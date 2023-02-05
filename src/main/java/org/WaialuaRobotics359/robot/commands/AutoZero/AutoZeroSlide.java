package org.WaialuaRobotics359.robot.commands.AutoZero;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Slide;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoZeroSlide extends CommandBase {
    private Slide s_Slide;

    private double currentLimit = 10;

    public AutoZeroSlide(Slide s_Slide) {
        this.s_Slide = s_Slide;
        addRequirements(s_Slide);
    }

    @Override
    public void initialize() {
        s_Slide.SetPosition(Constants.Slide.forwardSoftLimit +1000);
    }

    @Override
    public void execute() {
        s_Slide.SetPrecentOut(-.2);
    }
    
    @Override
    public boolean isFinished(){
        return s_Slide.getCurrent() > currentLimit;
    }

    @Override 
    public void end(boolean interupted) {
        s_Slide.Stop();
        s_Slide.SetHomePosition();
        s_Slide.setDesiredPosition(0);
    }
}

