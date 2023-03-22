package org.WaialuaRobotics359.robot.commands.autonomous;


import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

public class AutoIntakeCubeTime extends CommandBase {
    private Intake s_intake;

    private double currentLimit = 30;
    private Timer Timer = new Timer();

    public AutoIntakeCubeTime(Intake s_intake) {
        this.s_intake = s_intake;
        addRequirements(s_intake);
    }

    @Override
    public void initialize() {
        Timer.reset();
    }

    @Override
    public void execute() {
        s_intake.outake(Constants.Intake.speedIn);

        if (s_intake.getCurrent() > currentLimit){
            Timer.start();
        }
    }
    
    @Override
    public boolean isFinished(){
        return Timer.hasElapsed(2);
    }

    @Override 
    public void end(boolean interupted) {
        s_intake.stop();
    }
}