package org.WaialuaRobotics359.robot.commands;

import java.util.function.DoubleSupplier;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Slide;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualSlide extends CommandBase {
    private Slide s_Slide;
    private DoubleSupplier SlideAxis;
    
    private int positionIncrement = 10000;

    public ManualSlide(Slide s_Slide, DoubleSupplier SlideAxis) {
        this.s_Slide = s_Slide;
        this.SlideAxis = SlideAxis;
        addRequirements(s_Slide);
    }

    public void initialize(){

    }

    @Override
    public void execute(){

        //joystick control 
        double joystickValue = MathUtil.applyDeadband(SlideAxis.getAsDouble(), Constants.stickDeadband);
        if (Math.abs(joystickValue) > 0){
            
        s_Slide.incrementTargetPosition((int) (joystickValue * positionIncrement));
        s_Slide.setSlidePosition();

        }
    }
    @Override
    public boolean isFinished(){
        return false;
    }

    public void end(){

    }

    public void interrupted(){

    }
    
}