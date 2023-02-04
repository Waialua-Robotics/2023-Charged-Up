package org.WaialuaRobotics359.robot.commands.AutoZero;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoZeroWrist extends CommandBase {
    private Wrist s_Wrist;
    private Timer Timer = new Timer();
    private double currentLimit = 25;
    private double velocityChange = 300;
    private double timeChange = .3;

    public AutoZeroWrist(Wrist s_Wrist) {
        this.s_Wrist = s_Wrist;
        addRequirements(s_Wrist);
    }

    @Override
    public void initialize() {
        s_Wrist.SetPosition(Constants.Wrist.forwardSoftLimit +1000);
        
        Timer.reset();

        Timer.start();

    }

    @Override
    public void execute() {
        s_Wrist.SetPrecentOut(-.1);
    }
    
    @Override
    public boolean isFinished(){
        return s_Wrist.getCurrent() > currentLimit && Math.abs(s_Wrist.GetVelocity()) < velocityChange && Timer.hasElapsed(timeChange);
    }

    @Override 
    public void end(boolean interupted) {
        s_Wrist.Stop();
        s_Wrist.SetPosition(-1000);
        s_Wrist.setDesiredPosition(Constants.Wrist.StowPosition);
        s_Wrist.goToPosition();
    }
}