package org.WaialuaRobotics359.robot.commands.manual;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Fork;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualFork extends CommandBase {
    private Fork s_Fork;
    private DoubleSupplier ForkLift;
    private DoubleSupplier ForkLower;
    private BooleanSupplier Deploy;

    public ManualFork(Fork s_Fork, DoubleSupplier ForkLift, DoubleSupplier ForkLower, BooleanSupplier Deploy) {
        this.s_Fork = s_Fork;
        this.ForkLift = ForkLift;
        this.ForkLower = ForkLower;
        this.Deploy = Deploy;

        addRequirements(s_Fork);
    }

    Boolean ControlEnable;

    public void initialize() {
        ControlEnable = false;
    }

    
    public void execute(){

        Boolean deploystate= Deploy.getAsBoolean();

        double LiftTrigger = MathUtil.applyDeadband(ForkLift.getAsDouble(), Constants.OI.deadband);
        double LowerTrigger = MathUtil.applyDeadband(ForkLower.getAsDouble(), Constants.OI.deadband);

        if (deploystate){
            ControlEnable = true;
            s_Fork.openLatch();
        }

        if(ControlEnable){
            if(LiftTrigger>0) {
                s_Fork.SetPrecentOut(LiftTrigger);
            } else if (LowerTrigger>0) {
                s_Fork.SetPrecentOut(-LowerTrigger);
            } else {
                s_Fork.Stop();  
            }
        }
    }
            
    public boolean isFinished() { 
        return false;
    }

}