package org.WaialuaRobotics359.robot.subsystems;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private TalonFX mElevatorMotorL;
    private TalonFX mElevatorMotorR;
    private DigitalInput mMagSwitch;

    private int desiredPosition = 0;

    public boolean HasSwitched = false; 

    /*Logging*/
    private DataLog logger;

    /*elevator logs*/
    private DoubleLogEntry elevatorMotorDesiredPosition;
    private DoubleLogEntry elevatorMotorCurrentPosition;
    private DoubleLogEntry elevatorMotorVelocity;
    private DoubleLogEntry elevatorMotorLBusVoltage;
    private DoubleLogEntry elevatorMotorRBusVoltage;
    private DoubleLogEntry elevatorMotorLTemperature;
    private DoubleLogEntry elevatorMotorRTemperature;
    private BooleanLogEntry elevatorMagSwitch;
    private BooleanLogEntry elevatorInRange;

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

        /*Logging */
        logger = DataLogManager.getLog();

        elevatorMotorDesiredPosition = new DoubleLogEntry(logger, "elevator/desiredPosition");
        elevatorMotorCurrentPosition = new DoubleLogEntry(logger, "elevator/currentPosition");
        elevatorMotorVelocity = new DoubleLogEntry(logger, "elevator/velocity");
        elevatorMotorLTemperature = new DoubleLogEntry(logger, "elevator/MotorL/temperature");
        elevatorMotorRTemperature = new DoubleLogEntry(logger, "elevator/MotorR/temperature");
        elevatorMotorLBusVoltage = new DoubleLogEntry(logger, "elevator/MotorL/busVoltage");
        elevatorMotorRBusVoltage = new DoubleLogEntry(logger, "elevator/MotorR/busVoltage");
        elevatorMagSwitch = new BooleanLogEntry(logger, "elevator/magSwitch");
        elevatorInRange = new BooleanLogEntry(logger, "elevator/inRange");
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

    public void CurrentDisired(){
        mElevatorMotorR.set(TalonFXControlMode.MotionMagic, mElevatorMotorR.getSelectedSensorPosition());
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

    private void LogData(long time){
        elevatorMotorDesiredPosition.append(desiredPosition, time);
        elevatorMotorCurrentPosition.append(mElevatorMotorR.getSelectedSensorPosition(), time);
        elevatorMotorVelocity.append(mElevatorMotorR.getSelectedSensorVelocity(), time);
        elevatorMotorLTemperature.append(mElevatorMotorL.getTemperature(), time);
        elevatorMotorRTemperature.append(mElevatorMotorR.getTemperature(), time);
        elevatorMotorLBusVoltage.append(mElevatorMotorL.getBusVoltage(), time);
        elevatorMotorRBusVoltage.append(mElevatorMotorR.getBusVoltage(), time);
        elevatorMagSwitch.append(mMagSwitch.get(), time);
        elevatorInRange.append(inRange(), time);
    }

    public void periodic(){
        LogData(RobotController.getFPGATime());
    }
}
