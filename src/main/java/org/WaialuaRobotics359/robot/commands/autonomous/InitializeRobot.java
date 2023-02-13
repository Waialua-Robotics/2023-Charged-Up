package org.WaialuaRobotics359.robot.commands.autonomous;

import org.WaialuaRobotics359.robot.subsystems.Elevator;
import org.WaialuaRobotics359.robot.subsystems.Slide;
import org.WaialuaRobotics359.robot.subsystems.Swerve;
import org.WaialuaRobotics359.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class InitializeRobot extends CommandBase {
    @SuppressWarnings("unused") private Wrist s_Wrist;
    @SuppressWarnings("unused") private Elevator s_Elevator;
    @SuppressWarnings("unused") private Slide s_Slide;
    private Swerve s_Swerve;
    public InitializeRobot(Wrist s_Wrist, Elevator s_Elevator, Slide s_Slide, Swerve s_Swerve) {
        this.s_Wrist = s_Wrist;
        this.s_Elevator = s_Elevator;
        this.s_Slide = s_Slide;
        this.s_Swerve = s_Swerve;

        addRequirements(s_Wrist);
        addRequirements(s_Elevator);
        addRequirements(s_Slide);
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        //s_Swerve.setGyroYaw(((180 - s_Swerve.getYaw360()) % 360)); #TODO Remove for not spin spin
        s_Swerve.desiredAngle = s_Swerve.getYaw360();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
