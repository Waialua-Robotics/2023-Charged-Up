package org.WaialuaRobotics359.robot.subsystems;


import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    private TalonFX mIntakeMotor;
    public Intake() {
        mIntakeMotor = new TalonFX(Constants.Intake.intakeID);

        mIntakeMotor.configFactoryDefault();
        mIntakeMotor.configAllSettings(Robot.ctreConfigs.intakeFXConfig);
        mIntakeMotor.setInverted(Constants.Intake.intakeMotorInvert);
        mIntakeMotor.setNeutralMode(Constants.Intake.intakeNeutralMode);
    }

    public void intake() {
        mIntakeMotor.set(ControlMode.PercentOutput, Constants.Intake.speed);
    }

    public void outake() {
        mIntakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.speed);
    }

    public void stop() {
        mIntakeMotor.set(ControlMode.PercentOutput, 0);
    }

}