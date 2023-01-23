package org.WaialuaRobotics359.robot.commands.setPoints;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Wrist;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetWrist90PositionDegrees extends CommandBase {

    private Wrist s_Wrist;
    private double degrees;


    public SetWrist90PositionDegrees(Wrist s_Wrist) {
        this.s_Wrist = s_Wrist;
        addRequirements(s_Wrist);
        
    }

   
    
    public void initialize(){

    }

    
    @Override
    public void execute(){

        s_Wrist.setDesiredPosition(Constants.Wrist.Wrist90PositionDegrees);
        s_Wrist.goToPosition();

    }
    
    public boolean isFinished(){
        return true;
    }
}