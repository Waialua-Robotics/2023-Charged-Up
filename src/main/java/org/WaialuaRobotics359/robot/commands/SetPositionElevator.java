package org.WaialuaRobotics359.robot.commands;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetPositionElevator extends CommandBase {
    private Elevator s_Elevator;
    private int position;

    public SetPositionElevator(Elevator s_Elevator, int position) {
        this.s_Elevator = s_Elevator;
        this.position = position;
        addRequirements(s_Elevator);
    }

    public void initialize(){

    }

    @Override
    public void execute(){
    
    s_Elevator.setPosition(position);
    s_Elevator.setElevatorPosition();

    }
    
    public boolean isFinished(){
        return true;
    }
}
