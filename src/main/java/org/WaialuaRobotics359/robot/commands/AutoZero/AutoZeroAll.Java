package org.WaialuaRobotics359.robot.commands.AutoZero;

import org.WaialuaRobotics359.robot.subsystems.Slide;
import org.WaialuaRobotics359.robot.subsystems.Elevator;
import org.WaialuaRobotics359.robot.subsystems.Wrist;
import org.WaialuaRobotics359.robot.util.Dashboard;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.WaialuaRobotics359.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;

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

    boolean finished = false; 

    private Timer Timer = new Timer();

    public void initialize(){

        finished = false;
        
        Timer.reset();

        Timer.start();

        CommandScheduler.getInstance().schedule(m_AutoZeroWrist);
        CommandScheduler.getInstance().schedule(m_AutoZeroElevator);
        CommandScheduler.getInstance().schedule(m_AutoZeroSlide);


    }

    @Override
    public void execute(){

       SmartDashboard.putBoolean("AutoZeroEle",m_AutoZeroElevator.isFinished());

        if (m_AutoZeroWrist.isFinished() && m_AutoZeroElevator.isFinished() && m_AutoZeroSlide.isFinished()){
            finished = true;
        } 
        
    }
    
    public boolean isFinished(){
        return finished || true;
    }

    @Override 
    public void end(boolean interupted) {
        CommandScheduler.getInstance().cancel(m_AutoZeroWrist);
        CommandScheduler.getInstance().cancel(m_AutoZeroElevator);
        CommandScheduler.getInstance().cancel(m_AutoZeroSlide);

        if (Timer.hasElapsed(5)){
            System.out.println("AutoZero timed Out");
            DriverStation.reportWarning("AutoZeroAll timed out", false);
        }
    }
}