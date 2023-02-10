package org.WaialuaRobotics359.robot.commands.AutoZero;

import org.WaialuaRobotics359.robot.subsystems.Slide;
import org.WaialuaRobotics359.robot.subsystems.Elevator;
import org.WaialuaRobotics359.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;

public class AutoZeroAll extends CommandBase {
    private Wrist s_Wrist;
    private Elevator s_Elevator;
    private Slide s_Slide;

    private AutoZeroWrist m_AutoZeroWrist;
    private AutoZeroElevator m_AutoZeroElevator;
    private AutoZeroSlide m_AutoZeroSlide;

    public AutoZeroAll(Wrist s_Wrist, Elevator s_Elevator, Slide s_Slide) {
        this.s_Wrist = s_Wrist;
        this.s_Elevator = s_Elevator;
        this.s_Slide = s_Slide;
        this.m_AutoZeroWrist = new AutoZeroWrist(s_Wrist);
        this.m_AutoZeroElevator = new AutoZeroElevator(s_Elevator);
        this.m_AutoZeroSlide = new AutoZeroSlide(s_Slide);
        addRequirements(s_Wrist);
        addRequirements(s_Elevator);
        addRequirements(s_Slide);
    }

    public void initialize(){

        CommandScheduler.getInstance().schedule(m_AutoZeroWrist);
        CommandScheduler.getInstance().schedule(m_AutoZeroElevator);
        CommandScheduler.getInstance().schedule(m_AutoZeroSlide);


    }
    
    public boolean isFinished(){
        System.out.println("AutoZeroScheduled");
        DriverStation.reportWarning("AutoZeroing",false);
        return true;
    }

}