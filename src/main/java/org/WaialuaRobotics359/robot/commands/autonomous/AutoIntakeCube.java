package org.WaialuaRobotics359.robot.commands.autonomous;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Intake;

public class AutoIntake extends CommandBase {
    private Intake s_intake;

    public AutoIntake(Intake s_intake) {
        this.s_intake = s_intake;
        addRequirements(s_intake);
    }

    public void initialize() {}

    @Override
    public void execute(){

    

    }
    
    public boolean isFinished(){
        return s_intake.;
    }
}
