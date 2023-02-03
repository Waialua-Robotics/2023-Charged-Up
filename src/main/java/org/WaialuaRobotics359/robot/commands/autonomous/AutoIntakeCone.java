package org.WaialuaRobotics359.robot.commands.autonomous;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoIntakeCone extends CommandBase {
    private Intake s_intake;

    private double currentLimit = 30;

    public AutoIntakeCone(Intake s_intake) {
        this.s_intake = s_intake;
        addRequirements(s_intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        s_intake.outake(Constants.Intake.speed);
    }
    
    @Override
    public boolean isFinished(){
        return s_intake.getCurrent() > currentLimit;
    }

    @Override 
    public void end(boolean interupted) {
        s_intake.stop();
    }
}