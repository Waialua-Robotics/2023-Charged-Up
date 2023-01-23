package org.WaialuaRobotics359.robot.commands.setPoints;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetLowElevatorPositionInches extends CommandBase {

    private Elevator s_Elevator;
    private double inches;


    public SetLowElevatorPositionInches(Elevator s_Elevator) {
        this.s_Elevator = s_Elevator;
        addRequirements(s_Elevator);
        
    }

   
    
    public void initialize(){

    }

    
    @Override
    public void execute(){

        s_Elevator.setDesiredPosition(Constants.Elevator.LowElevatorPositionInches);
        s_Elevator.goToPosition();

    }
    
    public boolean isFinished(){
        return true;
    }
}
    
