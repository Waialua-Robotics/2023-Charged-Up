package org.WaialuaRobotics359.robot.commands;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Slide;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetPositionSlide extends CommandBase {
    private Slide s_Slide;
    private int position;

    public SetPositionSlide(Slide s_Slide, int position) {
        this.s_Slide = s_Slide;
        this.position = position;
        addRequirements(s_Slide);
    }

    public void initialize(){

    }

    @Override
    public void execute(){
    
    s_Slide.setPosition(position);
    s_Slide.setSlidePosition();

    }
    
    public boolean isFinished(){
        return true;
    }
}

