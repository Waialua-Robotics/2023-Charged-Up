package org.WaialuaRobotics359.robot.subsystems;


import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
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

    public void SetWristPosition() {
        mWristMotor.set(TalonFXControlMode.Position, desiredPosition);
    }

    public void incrementTargetPosition(int increment) {
        int Current = getPosition();
        int newDesired = Current + increment;
        if (isValidPosition(newDesired)) {
            desiredPosition = newDesired; 
        }
    }
    public boolean isValidPosition(int position) {
        boolean valid = position >= Constants.Wrist.minHeight && position <= Constants.Wrist.maxHeight;
        return valid;
    }
}
