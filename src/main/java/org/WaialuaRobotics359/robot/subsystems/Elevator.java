package org.WaialuaRobotics359.robot.subsystems;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private TalonFX mElevatorMotorL;
    private TalonFX mElevatorMotorR;
    private DigitalInput mMagSwitch;

    private int desiredPosition = 0;

    public Elevator () {
        mElevatorMotorL = new TalonFX(Constants.Elevator.lElevatorID);
        mElevatorMotorR = new TalonFX(Constants.Elevator.rElevatorID);

        mElevatorMotorR.configFactoryDefault();
        mElevatorMotorL.configFactoryDefault();
        
        mElevatorMotorR.configAllSettings(Robot.ctreConfigs.elevatorFXConfig);

        mElevatorMotorL.set(ControlMode.Follower, Constants.Elevator.rElevatorID);
        mElevatorMotorL.setInverted(TalonFXInvertType.OpposeMaster);

        mElevatorMotorR.setSelectedSensorPosition(0);

        mMagSwitch = new DigitalInput(Constants.Elevator.MagElevatorID);
    }

    public void setDesiredPosition(int position) {
        desiredPosition = fitToRange(position);
    }

    public int getDesiredPosition() {
        return desiredPosition;
    }

    public void currentToDisired(){
        setDesiredPosition(GetPosition());
    }

    public void goToPosition() {
        mElevatorMotorR.set(TalonFXControlMode.MotionMagic, desiredPosition);
    }

    public boolean inRange() {
        int encoder = (int) mElevatorMotorR.getSelectedSensorPosition();
        return (encoder > (desiredPosition - Constants.Elevator.threshold)) && (encoder < (desiredPosition + Constants.Elevator.threshold));
    }

    public int GetPosition() {
        return (int) mElevatorMotorR.getSelectedSensorPosition();
    }

    public double GetPositionInches(){
        return  (GetPosition()*(Constants.Elevator.Ratio))/61440;
    }

    public double GetVelocity(){
        return mElevatorMotorR.getSelectedSensorVelocity();
    }

    public double getCurrent(){
        return mElevatorMotorR.getStatorCurrent();
    }

    public void SetPosition(double position){
        mElevatorMotorR.setSelectedSensorPosition(position);
    }

    public void SetHomePosition(){
        mElevatorMotorR.setSelectedSensorPosition(0);
    }

    public void Stop(){
        SetPrecentOut(0);
    }

    public void SetPrecentOut(double percent){
        mElevatorMotorR.set(TalonFXControlMode.PercentOutput, percent);
    }

    public boolean getSwitch(){
        return !mMagSwitch.get();
    }


    private int fitToRange(int position) {
        desiredPosition = Math.min(position, Constants.Elevator.forwardSoftLimit);
        desiredPosition = Math.max(desiredPosition, Constants.Elevator.reverseSoftLimit);
        return desiredPosition;
    }

}
