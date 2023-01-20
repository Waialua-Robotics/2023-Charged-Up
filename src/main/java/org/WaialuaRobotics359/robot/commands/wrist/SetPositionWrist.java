package org.WaialuaRobotics359.robot.commands.wrist;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetPositionWrist extends CommandBase {
    private Wrist s_Wrist;
    private int position;

    public SetPositionWrist(Wrist s_Wrist, int position) {
        this.s_Wrist = s_Wrist;
        this.position = position;
        addRequirements(s_Wrist);
    }

    public void initialize(){

    }

    @Override
    public void execute(){
    
        s_Wrist.setDesiredPosition(position);
        s_Wrist.goToPosition();

    }
    
    public boolean isFinished(){
        return true;
    }
}
