package org.WaialuaRobotics359.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class Fork extends SubsystemBase {
    private TalonFX mForkMotor;
    Servo leftServo;
    Servo rightServo;

    public Fork() {
        mForkMotor = new TalonFX(Constants.Fork.forkMotorID);

        mForkMotor.configFactoryDefault();
        mForkMotor.configAllSettings(Robot.ctreConfigs.forkFXConfig);
        mForkMotor.setInverted(Constants.Fork.forkMotorInvert);
        mForkMotor.setNeutralMode(Constants.Fork.forkNeutralMode);

        leftServo = new Servo(0);
        rightServo = new Servo(1);

        leftServo.set(1);
        rightServo.set(0);
    }

    public void openLatch() {
        leftServo.set(.25);
        rightServo.set(.75);
    }

    public void closeLatch() {
        leftServo.set(1);
        rightServo.set(0);
    }

    public void SetPrecentOut(double percent){
        mForkMotor.set(TalonFXControlMode.PercentOutput, percent);
    }

    public void Stop(){
        SetPrecentOut(0);
    }
}

