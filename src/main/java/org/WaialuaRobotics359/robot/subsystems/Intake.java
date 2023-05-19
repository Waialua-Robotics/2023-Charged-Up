package org.WaialuaRobotics359.robot.subsystems;


import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    private TalonFX mIntakeMotor;
    private StringLogEntry myStringLog;
    public Intake() {
        mIntakeMotor = new TalonFX(Constants.Intake.intakeID);

        mIntakeMotor.configFactoryDefault();
        mIntakeMotor.configAllSettings(Robot.ctreConfigs.intakeFXConfig);
        mIntakeMotor.setInverted(Constants.Intake.intakeMotorInvert);
        mIntakeMotor.setNeutralMode(Constants.Intake.intakeNeutralMode);
        DataLog log = DataLogManager.getLog();
        myStringLog = new StringLogEntry(log, "/my/string");
    }

    public void intake(double intakeSpeed) {
        mIntakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
        myStringLog.append("intakeSpeed: " + intakeSpeed);
    }

    public void outake(double intakeSpeed) {
        mIntakeMotor.set(ControlMode.PercentOutput, -intakeSpeed);
    }

    public double getCurrent() {
        return mIntakeMotor.getStatorCurrent();
    }

    public void stop() {
        mIntakeMotor.set(ControlMode.PercentOutput, 0);
    }

}