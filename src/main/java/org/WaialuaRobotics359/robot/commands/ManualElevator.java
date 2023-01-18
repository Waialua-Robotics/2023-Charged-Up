package org.WaialuaRobotics359.robot.commands;

import java.util.function.DoubleSupplier;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualElevator extends CommandBase {
    private Elevator s_Elevator;
    private DoubleSupplier elevatorAxis;
    
    private int positionIncrement = 300;

    public ManualElevator(Elevator s_Elevator, DoubleSupplier elevatorAxis) {
        this.s_Elevator = s_Elevator;
        addRequirements(s_Elevator);

        this.elevatorAxis = elevatorAxis;
    }

    public void initialize(){

    }

    public void execute(){
        //joystick control 
        double joystickVal = MathUtil.applyDeadband(elevatorAxis.getAsDouble(), Constants.stickDeadband);
        s_Elevator.incrementTargetPosition((int) (joystickVal * positionIncrement));

        s_Elevator.setElevatorPosition();
    }

    public boolean isFinished(){
        return false;
    }

    public void end(){

    }

    public void interrupted(){

    }
    
}
