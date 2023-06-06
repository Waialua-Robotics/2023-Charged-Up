package org.WaialuaRobotics359.robot.subsystems;


import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    private TalonFX mIntakeMotor;
    public TalonFXConfigurator intakeFXConfigurator = mIntakeMotor.getConfigurator();
    
    /* Percent Output */
    private DutyCycleOut cyclerequest = new DutyCycleOut(0.0);

    /*Logging*/
    private DataLog logger;

    /*elevator logs*/
    private DoubleLogEntry intakeMotorCurrent;
    private DoubleLogEntry intakeMotorTemperature;
    private DoubleLogEntry  intakeMotorOutputPercent;
    private DoubleLogEntry intakeMotorVelocity;

    public Intake() {
        mIntakeMotor = new TalonFX(Constants.Intake.intakeID);

        mIntakeMotor.getConfigurator().apply(Robot.ctreConfigs.intakeFXConfig);
        mIntakeMotor.setInverted(Constants.Intake.intakeMotorInvert);

        /*Logging*/
        logger = DataLogManager.getLog();
        intakeMotorCurrent = new DoubleLogEntry(logger, "intake/motorCurrent");
        intakeMotorTemperature = new DoubleLogEntry(logger, "intake/motorTemperature");
        intakeMotorOutputPercent = new DoubleLogEntry(logger, "intake/motorOutputPercent");
        intakeMotorVelocity = new DoubleLogEntry(logger, "intake/motorVelocity");
    }

    public void intake(double intakeSpeed) {
        mIntakeMotor.setControl(cyclerequest.withOutput(intakeSpeed));
    }

    public void outake(double intakeSpeed) {
        mIntakeMotor.setControl(cyclerequest.withOutput(-intakeSpeed));
    }

    public double getCurrent() {
        return mIntakeMotor.getStatorCurrent().getValue();
    }

    public double getPercentOutput() {
        return mIntakeMotor.getDutyCycle().getValue();
    }

    public void stop() {
        mIntakeMotor.setControl(cyclerequest.withOutput(0));
    }

    private void logData(long time){
        intakeMotorCurrent.append(getCurrent(), time);
        intakeMotorTemperature.append(mIntakeMotor.getDeviceTemp().getValue(), time);
        intakeMotorOutputPercent.append(getPercentOutput(), time);
        intakeMotorVelocity.append(mIntakeMotor.getRotorVelocity().getValue(), time);
    }

    public void periodic() {
        logData(RobotController.getFPGATime());
    }

}