package org.WaialuaRobotics359.robot.subsystems;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Elevator {
    private TalonFX mElevatorMotorL;
    private TalonFX mElevatorMotorR;

    private int desiredPosition;

    public Elevator (int rightID, int leftID) {
        mElevatorMotorL = new TalonFX(leftID);
        mElevatorMotorR = new TalonFX(rightID);

        mElevatorMotorR.configFactoryDefault();
        mElevatorMotorL.configFactoryDefault();
        
        mElevatorMotorR.configAllSettings(Robot.ctreConfigs.elevatorFXConfig);

        mElevatorMotorL.set(ControlMode.Follower, Constants.Elevator.rElevatorID);
        mElevatorMotorL.setInverted(TalonFXInvertType.OpposeMaster);
    }

    public void setPosition(int position) {
        desiredPosition = position;
    }

    public int getPosition() {
        return desiredPosition;
    }

    public boolean inRange() {
        int encoder = (int) mElevatorMotorR.getSelectedSensorPosition();
        return (encoder > (desiredPosition - Constants.Elevator.threshold)) && (encoder < (desiredPosition + Constants.Elevator.threshold));
    }

    public void periodic() {
        mElevatorMotorR.set(TalonFXControlMode.Position, desiredPosition);
    }


}
