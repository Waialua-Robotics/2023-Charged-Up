package org.WaialuaRobotics359.robot.commands.autonomous;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Intake;
import org.WaialuaRobotics359.robot.subsystems.Slide;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoIntakeConeSlide extends CommandBase {
    private Intake s_intake;
    private Slide s_slide;

    private double currentLimit = 30;
    private double slidePose;

    public AutoIntakeConeSlide(Intake s_intake, Slide s_slide) {
        this.s_intake = s_intake;
        this.s_slide = s_slide;
        addRequirements(s_intake);
        addRequirements(s_slide);
    }

    @Override
    public void initialize() {
        slidePose = s_slide.GetPosition();
    }

    @Override
    public void execute() {
        s_intake.intake(Constants.Intake.speedIn);

        slidePose += 1000;
        s_slide.setDesiredPosition(slidePose);
        s_slide.goToPosition();
        

    }
    
    @Override
    public boolean isFinished(){
        return s_intake.getCurrent() > currentLimit;
    }

    @Override 
    public void end(boolean interupted) {
        s_intake.stop();
        s_slide.setDesiredPosition(Constants.Slide.Cone.LowPosition);
        s_slide.goToPosition();
    }
}