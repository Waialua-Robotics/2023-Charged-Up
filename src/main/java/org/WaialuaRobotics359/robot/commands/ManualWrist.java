package org.WaialuaRobotics359.robot.commands;

import java.util.function.DoubleSupplier;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualWrist extends CommandBase {
    private Wrist s_Wrist;
    private DoubleSupplier WristAxis;
    
    private int positionIncrement = 10000;

    public ManualWrist(Wrist s_Wrist, DoubleSupplier WristAxis) {
        this.s_Wrist = s_Wrist;
        this.WristAxis = WristAxis;
        addRequirements(s_Wrist);
    }

    public void initialize(){

    }

    @Override
    public void execute(){

        //joystick control 
        double joystickValue = MathUtil.applyDeadband(WristAxis.getAsDouble(), Constants.stickDeadband);
        if (Math.abs(joystickValue) > 0){
            
        s_Wrist.incrementTargetPosition((int) (joystickValue * positionIncrement));
        s_Wrist.SetWristPosition();

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