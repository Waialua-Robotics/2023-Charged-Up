package org.WaialuaRobotics359.robot.subsystems;


import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private TalonFX mWristMotor;
    private int desiredPosition = 0;

    /*Logging*/
    private DataLog logger;
    /*elevator logs*/
    private DoubleLogEntry wristDesiredPosition;
    private DoubleLogEntry wristCurrentPosition;
    private DoubleLogEntry wristMotorCurrent;
    private DoubleLogEntry wristMotorVelocity;
    private DoubleLogEntry wristMotorTemperature;
    private BooleanLogEntry wristInSafe;
    
    public Wrist() {
        mWristMotor = new TalonFX(Constants.Wrist.wristID);

        mWristMotor.getConfigurator().apply(new TalonFXConfiguration());
        mWristMotor.configAllSettings(Robot.ctreConfigs.wristFXConfig);
        mWristMotor.setInverted(Constants.Wrist.wristMotorInvert);
        mWristMotor.setNeutralMode(Constants.Wrist.wristNeutralMode);
        mWristMotor.setRotorPosition(0);

        /*Logging*/
        logger = DataLogManager.getLog();
        wristDesiredPosition = new DoubleLogEntry(logger, "wrist/desiredPosition");
        wristCurrentPosition = new DoubleLogEntry(logger, "wrist/currentPosition");
        wristMotorCurrent = new DoubleLogEntry(logger, "wrist/motorCurrent");
        wristMotorVelocity = new DoubleLogEntry(logger, "wrist/motorVelocity");
        wristMotorTemperature = new DoubleLogEntry(logger, "wrist/motorTemperature");
        wristInSafe = new BooleanLogEntry(logger, "wrist/inSafe");
    }

 
    public void setDesiredPosition(double d) {
        desiredPosition = fitToRange(d);
    }

    public int getDesiredPosition() {
        return desiredPosition;
    }

    public void currentToDisired(){
        setDesiredPosition((int)GetPosition());
    }


    public void goToPosition() {
        mWristMotor.set(TalonFXControlMode.MotionMagic, desiredPosition);
    }

    public boolean inRange() {
        double encoder = mWristMotor.getRotorPosition().getValue();
        return (encoder > (desiredPosition - Constants.Wrist.threshold)) && (encoder < (desiredPosition + Constants.Wrist.threshold));
    }

    public boolean inSafe(){
        double encoder = mWristMotor.getRotorPosition().getValue();
        return (encoder > (Constants.Wrist.SafePosition - Constants.Wrist.threshold)) && (encoder < (desiredPosition + Constants.Wrist.threshold));
    }

    public double GetPosition() {
        return mWristMotor.getRotorPosition().getValue();
     }

     public double GetPositionInches(){
        return  (GetPosition()*(Constants.Wrist.Ratio))/61440;
     }

     public double GetVelocity(){
        return mWristMotor.getRotorVelocity().getValue();
    }

     public void SetPosition(double position){
        mWristMotor.setRotorPosition(position);
     }

     public void Stop(){
        SetPrecentOut(0);
    }

    public double getCurrent(){
        return mWristMotor.getStatorCurrent().getValue();
    }


    public void SetPrecentOut(double percent){
        mWristMotor.set(TalonFXControlMode.PercentOutput, percent);
    }

    public void SetHomePosition(){
        mWristMotor.setRotorPosition(0);
    }

    private int fitToRange(int position) {
        desiredPosition = Math.min(position, Constants.Wrist.forwardSoftLimit);
        desiredPosition = Math.max(desiredPosition, Constants.Wrist.reverseSoftLimit);
        return desiredPosition;
    }

    private void LogData(long time){
        wristDesiredPosition.append(desiredPosition, time);
        wristCurrentPosition.append(GetPosition(), time);
        wristMotorCurrent.append(getCurrent(), time);
        wristMotorVelocity.append(GetVelocity(), time);
        wristMotorTemperature.append(mWristMotor.getDeviceTemp().getValue(), time);
        wristInSafe.append(inSafe(), time);
    }

    public void periodic(){
        LogData(RobotController.getFPGATime());
    }
}