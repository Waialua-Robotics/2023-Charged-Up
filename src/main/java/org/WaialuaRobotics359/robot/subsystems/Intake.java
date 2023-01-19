package org.WaialuaRobotics359.robot.subsystems;


import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    private TalonFX mIntakeMotor;
    public Intake(int intakeID) {
        mIntakeMotor = new TalonFX(intakeID);

        mIntakeMotor.configFactoryDefault();
        mIntakeMotor.configAllSettings(Robot.ctreConfigs.intakeFXConfig);
        mIntakeMotor.setInverted(Constants.Intake.intakeMotorInvert);
        mIntakeMotor.setNeutralMode(Constants.Intake.intakeNeutralMode);
    }

    public void intakeChomp(double value) {
        mIntakeMotor.set(ControlMode.PercentOutput, value);
    }

}