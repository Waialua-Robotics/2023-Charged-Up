package org.WaialuaRobotics359.robot.commands.AutoZero;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoZeroElevator extends CommandBase {
    private Elevator s_Elevator;
    private Timer Timer = new Timer();

    private double currentLimit = 5;
    private double velocityChange = 50;

    public AutoZeroElevator(Elevator s_Elevator) {
        this.s_Elevator = s_Elevator;
        addRequirements(s_Elevator);
    }

    @Override
    public void initialize() {
        s_Elevator.SetPosition(Constants.Elevator.forwardSoftLimit +1000);
          
        Timer.reset();
        Timer.start();

    }

    @Override
    public void execute() {
        s_Elevator.SetPrecentOut(-.1);
    }
    
    @Override
    public boolean isFinished(){
        return s_Elevator.getCurrent() > currentLimit && Math.abs(s_Elevator.GetVelocity()) < velocityChange && Timer.hasElapsed(.2);
    }

    @Override 
    public void end(boolean interupted) {
        s_Elevator.Stop();
        s_Elevator.SetHomePosition();
        s_Elevator.setDesiredPosition(0);
    }
}
