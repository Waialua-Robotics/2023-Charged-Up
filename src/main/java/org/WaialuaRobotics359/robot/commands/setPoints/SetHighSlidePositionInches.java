package org.WaialuaRobotics359.robot.commands.setPoints;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Slide;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetHighSlidePositionInches extends CommandBase {

    private Slide s_Slide;
    private double inches;


    public SetHighSlidePositionInches(Slide s_Slide) {
        this.s_Slide = s_Slide;
        addRequirements(s_Slide);
        
    }

   
    
    public void initialize(){

    }

    
    @Override
    public void execute(){

        s_Slide.setDesiredPosition(Constants.Slide.HighSlidePositionInches);
        s_Slide.goToPosition();

    }
    
    public boolean isFinished(){
        return true;
    }
}