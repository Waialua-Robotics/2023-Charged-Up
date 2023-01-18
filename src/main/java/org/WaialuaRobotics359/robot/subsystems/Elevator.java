package org.WaialuaRobotics359.robot.subsystems;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;
import org.WaialuaRobotics359.robot.commands.ManualElevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private TalonFX mElevatorMotorL;
    private TalonFX mElevatorMotorR;

    private int desiredPosition;

    public Elevator () {
        mElevatorMotorL = new TalonFX(Constants.Elevator.lElevatorID);
        mElevatorMotorR = new TalonFX(Constants.Elevator.rElevatorID);

        mElevatorMotorR.configFactoryDefault();
        mElevatorMotorL.configFactoryDefault();
        
        mElevatorMotorR.configAllSettings(Robot.ctreConfigs.elevatorFXConfig);

        mElevatorMotorL.set(ControlMode.Follower, Constants.Elevator.rElevatorID);
        mElevatorMotorL.setInverted(TalonFXInvertType.OpposeMaster);
    }

    public void setElevatorPosition () {
        mElevatorMotorR.set(TalonFXControlMode.Position, desiredPosition);
    }

    public boolean isValidPosition(int position) {
        boolean valid = position >= Constants.Elevator.minHeight && position <= Constants.Elevator.maxHeight;
        return valid;
    }

    public void incrementTargetPosition(int increment) {
        int Current = getPosition();
        int newDesired = Current + increment;
        if (isValidPosition(newDesired)) {
            desiredPosition = newDesired; 
        }
    }

    public void setPosition(int position) {
        desiredPosition = Math.min(position, Constants.Elevator.maxHeight);
        desiredPosition = Math.max(desiredPosition, Constants.Elevator.minHeight);

        mElevatorMotorR.set(TalonFXControlMode.Position, desiredPosition);
    }

    public void setEncodeerPosition(int position) {
        mElevatorMotorR.setSelectedSensorPosition(position);
    }

    public void setHold(){
        setPosition(getPosition());
    }

    public int getPosition() {
        return desiredPosition;
    }

    public int getEncoder() {
        return (int) mElevatorMotorR.getSelectedSensorPosition();
    }

    public boolean inRange() {
        int encoder = (int) mElevatorMotorR.getSelectedSensorPosition();
        return (encoder > (desiredPosition - Constants.Elevator.threshold)) && (encoder < (desiredPosition + Constants.Elevator.threshold));
    }

    public void setDefaultCommand(ManualElevator manualElevator) {
    }

}
