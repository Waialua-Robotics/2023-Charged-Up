package org.WaialuaRobotics359.robot.commands.manual;

import java.util.function.BooleanSupplier;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.RobotContainer;
import org.WaialuaRobotics359.robot.subsystems.Intake;
import org.WaialuaRobotics359.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualIntake extends CommandBase {
    private Intake s_Intake;
    private BooleanSupplier Intake;
    private BooleanSupplier Outake;

    private double currentLimit = 30;

    //private int percentIncrement = ;

    public ManualIntake(Intake s_Intake, BooleanSupplier Intake, BooleanSupplier Outake) {
        this.s_Intake = s_Intake;
        this.Intake = Intake;
        this.Outake = Outake;
        addRequirements(s_Intake);
    }

    double intakeSpeed;
    double outtakeSpeed;

    public void initialize(){}

    @Override
    public void execute(){

        if(RobotContainer.isCube){
            intakeSpeed = -Constants.Intake.speedIn;
            outtakeSpeed = -Constants.Intake.speedOut;
        } else {
            intakeSpeed = Constants.Intake.speedIn;
            outtakeSpeed = Constants.Intake.speedOut;
        }

        //trigger control 
        boolean rBumperValue = Intake.getAsBoolean();
        boolean lBumperValue = Outake.getAsBoolean();

        if(rBumperValue) {
            s_Intake.intake(intakeSpeed);
            if (s_Intake.getCurrent() > currentLimit){
                LEDs.hasObject = true;
            }
        } else if (lBumperValue) {
            s_Intake.outake(outtakeSpeed);
            LEDs.hasObject = false;
        } else {
            s_Intake.stop();  
        }
    }


}
