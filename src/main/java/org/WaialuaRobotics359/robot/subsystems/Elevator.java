package org.WaialuaRobotics359.robot.subsystems;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;


import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.hardware.TalonFX;

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
    private DutyCycleOut elevatormotor;
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

        mElevatorMotorR.getConfigurator().apply(new TalonFXConfiguration());
        mElevatorMotorL.getConfigurator().apply(new TalonFXConfiguration());
        
        mElevatorMotorR.configAllSettings(Robot.ctreConfigs.elevatorFXConfig);

        mElevatorMotorL.setControl(Follower)//(ControlMode.Follower, Constants.Elevator.rElevatorID);
        mElevatorMotorL.setInverted(TalonFXInvertType.OpposeMaster);

        mElevatorMotorR.setRotorPosition(0);

        mMagSwitch = new DigitalInput(Constants.Elevator.MagElevatorID);

        elevatormotor = new DutyCycleOut(0.0);

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
        mElevatorMotorR.set(TalonFXControlMode.MotionMagic, mElevatorMotorR.getRotorPosition());
    }

    public boolean inRange() {
        double encoder = mElevatorMotorR.getPosition().getValue();
        return (encoder > (desiredPosition - Constants.Elevator.threshold)) && (encoder < (desiredPosition + Constants.Elevator.threshold));
    }

    public double GetPosition() {
        return mElevatorMotorR.getPosition().getValue();
    }

    public double GetPositionInches(){
        return  (GetPosition()*(Constants.Elevator.Ratio))/61440;
    }

    public double GetVelocity(){
        return mElevatorMotorR.getVelocity().getValue();
    }

    public double getCurrent(){
        return mElevatorMotorR.getStatorCurrent().getValue();
    }

    public void SetPosition(double position){
        mElevatorMotorR.setRotorPosition(position);
    }

    public void SetHomePosition(){
        mElevatorMotorR.setRotorPosition(0);
    }

    public void Stop(){
        SetPrecentOut(0);
    }

    public void SetPrecentOut(double percent){
        mElevatorMotorR.setControl(elevatormotor.withOutput(percent));
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
        elevatorMotorCurrentPosition.append(mElevatorMotorR.getPosition().getValue(), time);
        elevatorMotorVelocity.append(mElevatorMotorR.getVelocity().getValue(), time);
        elevatorMotorLTemperature.append(mElevatorMotorL.getTemperature(), time);
        elevatorMotorRTemperature.append(mElevatorMotorR.getTemperature(), time);
        elevatorMotorLBusVoltage.append(mElevatorMotorL.getSupplyVoltage().getValue(), time);
        elevatorMotorRBusVoltage.append(mElevatorMotorR.getSupplyVoltage().getValue(), time);
        elevatorMagSwitch.append(mMagSwitch.get(), time);
        elevatorInRange.append(inRange(), time);
    }

    public void periodic(){
        LogData(RobotController.getFPGATime());
    }
}
