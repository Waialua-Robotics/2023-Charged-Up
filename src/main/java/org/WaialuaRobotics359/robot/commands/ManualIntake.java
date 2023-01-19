package org.WaialuaRobotics359.robot.commands;

import java.util.function.DoubleSupplier;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualIntake extends CommandBase {
    private Intake s_Intake;
    private DoubleSupplier IntakeAxis;
    private DoubleSupplier IntakeAxisN;

    //private int percentIncrement = ;

    public ManualIntake(Intake s_Intake, DoubleSupplier IntakeAxis, DoubleSupplier IntakeAxisN) {
        this.s_Intake = s_Intake;
        this.IntakeAxis = IntakeAxis;
        this.IntakeAxisN = IntakeAxisN;
        addRequirements(s_Intake);
    }

    public void initialize(){

    }
    

    @Override
    public void execute(){

        //trigger control 
        double rTriggerValue = IntakeAxis.getAsDouble();
        double lTriggerValue = IntakeAxisN.getAsDouble();

        if((Math.abs(rTriggerValue) > 0)) {
            s_Intake.intakeChomp(rTriggerValue);
        }

        else if((Math.abs(lTriggerValue) > 0)) {
            s_Intake.intakeChomp(-lTriggerValue);
        }

        else{
            s_Intake.intakeChomp(0);  
        }

    }


}
