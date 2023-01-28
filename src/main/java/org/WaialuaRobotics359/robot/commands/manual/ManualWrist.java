package org.WaialuaRobotics359.robot.commands.manual;

import java.util.function.DoubleSupplier;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualWrist extends CommandBase {
    private Wrist s_Wrist;
    private DoubleSupplier WristAxis;
    private DoubleSupplier WristAxisN;
    
    private int positionIncrement = 500;

    public ManualWrist(Wrist s_Wrist, DoubleSupplier WristAxis, DoubleSupplier WristAxisN) {
        this.s_Wrist = s_Wrist;
        this.WristAxis = WristAxis;
        this.WristAxisN = WristAxisN;
        addRequirements(s_Wrist);
    }

    public void initialize(){

    }

    @Override
    public void execute(){

        //joystick control 
        double rTriggerValue = MathUtil.applyDeadband(WristAxis.getAsDouble(), Constants.OI.deadband);
        double lTriggerValue = MathUtil.applyDeadband(WristAxisN.getAsDouble(), Constants.OI.deadband);

        if (Math.abs(rTriggerValue) > 0) {
            s_Wrist.setDesiredPosition( (int) (s_Wrist.getDesiredPosition() + (rTriggerValue * positionIncrement)) );
        } else if (Math.abs(lTriggerValue) > 0) {
            s_Wrist.setDesiredPosition( (int) (s_Wrist.getDesiredPosition() + (-lTriggerValue * positionIncrement)) );
        }

        s_Wrist.goToPosition();

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