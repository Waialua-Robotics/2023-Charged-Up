package org.WaialuaRobotics359.robot.subsystems;


import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Wrist {
    private TalonFX mWristMotor;
    private int desiredPosition;
    public Wrist(int wristID) {
        mWristMotor = new TalonFX(wristID);

        mWristMotor.configFactoryDefault();
        mWristMotor.configAllSettings(Robot.ctreConfigs.wristFXConfig);
        mWristMotor.setInverted(Constants.Wrist.wristMotorInvert);
        mWristMotor.setNeutralMode(Constants.Wrist.wristNeutralMode);
    }

    public void setPosition(int position) {
        desiredPosition = position;
    }


    public int getPosition() {
        return desiredPosition;
    }


    public boolean inRange() {
        int encoder = (int) mWristMotor.getSelectedSensorPosition();
        return (encoder > (desiredPosition - Constants.Wrist.threshold)) && (encoder < (desiredPosition + Constants.Wrist.threshold));
    }

  public void periodic() {
        mWristMotor.set(TalonFXControlMode.Position, desiredPosition);
    }


}
