package org.WaialuaRobotics359.robot.commands;

import java.util.function.DoubleSupplier;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualElevator extends CommandBase {
    private Elevator s_Elevator;
    private DoubleSupplier elevatorAxis;
    
    private int positionIncrement = 10000;

    public ManualElevator(Elevator s_Elevator, DoubleSupplier elevatorAxis) {
        this.s_Elevator = s_Elevator;
        this.elevatorAxis = elevatorAxis;
        addRequirements(s_Elevator);
    }

    public void initialize(){

    }

    @Override
    public void execute(){

        //joystick control 
        double joystickValue = MathUtil.applyDeadband(elevatorAxis.getAsDouble(), Constants.stickDeadband);
        if (Math.abs(joystickValue) > 0){
        s_Elevator.incrementTargetPosition((int) (joystickValue * positionIncrement));
        s_Elevator.setElevatorPosition();

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
