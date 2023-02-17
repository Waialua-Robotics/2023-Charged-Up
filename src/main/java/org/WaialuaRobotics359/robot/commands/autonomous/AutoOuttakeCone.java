package org.WaialuaRobotics359.robot.commands.autonomous;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoOuttakeCone extends CommandBase {
    private Intake s_intake;

    public AutoOuttakeCone(Intake s_intake) {
        this.s_intake = s_intake;
        addRequirements(s_intake);
    }

    private Timer Timer = new Timer();

    @Override
    public void initialize() {
        Timer.reset();
        Timer.start();
    }

    @Override
    public void execute() {
        s_intake.outake((.5)*Constants.Intake.speedIn);
    }
    
    @Override
    public boolean isFinished(){
        return Timer.hasElapsed(.5);
    }

    @Override 
    public void end(boolean interupted) {
        s_intake.stop();
    }
}