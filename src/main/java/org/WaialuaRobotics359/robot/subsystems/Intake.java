package org.WaialuaRobotics359.robot.subsystems;


import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    private TalonFX mIntakeMotor;

    /*Logging*/
    private DataLog logger;

    /*elevator logs*/
    private DoubleLogEntry intakeMotorCurrent;
    private DoubleLogEntry intakeMotorTemperature;
    private DoubleLogEntry  intakeMotorOutputPercent;
    private DoubleLogEntry intakeMotorVelocity;

    public Intake() {
        mIntakeMotor = new TalonFX(Constants.Intake.intakeID);

        mIntakeMotor.configFactoryDefault();
        mIntakeMotor.configAllSettings(Robot.ctreConfigs.intakeFXConfig);
        mIntakeMotor.setInverted(Constants.Intake.intakeMotorInvert);
        mIntakeMotor.setNeutralMode(Constants.Intake.intakeNeutralMode);

        /*Logging*/
        logger = DataLogManager.getLog();
        intakeMotorCurrent = new DoubleLogEntry(logger, "intake/motorCurrent");
        intakeMotorTemperature = new DoubleLogEntry(logger, "intake/motorTemperature");
        intakeMotorOutputPercent = new DoubleLogEntry(logger, "intake/motorOutputPercent");
        intakeMotorVelocity = new DoubleLogEntry(logger, "intake/motorVelocity");
    }

    public void intake(double intakeSpeed) {
        mIntakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
    }

    public void outake(double intakeSpeed) {
        mIntakeMotor.set(ControlMode.PercentOutput, -intakeSpeed);
    }

    public double getCurrent() {
        return mIntakeMotor.getStatorCurrent().getValue();
    }

    public double getPercentOutput() {
        return mIntakeMotor.getMotorOutputPercent();
    }

    public void stop() {
        mIntakeMotor.set(ControlMode.PercentOutput, 0);
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