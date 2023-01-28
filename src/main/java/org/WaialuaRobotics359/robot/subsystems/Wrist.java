package org.WaialuaRobotics359.robot.subsystems;


import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private TalonFX mWristMotor;
    private int desiredPosition = 0;
    
    public Wrist() {
        mWristMotor = new TalonFX(Constants.Wrist.wristID);

        mWristMotor.configFactoryDefault();
        mWristMotor.configAllSettings(Robot.ctreConfigs.wristFXConfig);
        mWristMotor.setInverted(Constants.Wrist.wristMotorInvert);
        mWristMotor.setNeutralMode(Constants.Wrist.wristNeutralMode);
        mWristMotor.setSelectedSensorPosition(0);
    }

 
    public void setDesiredPosition(int position) {
        desiredPosition = fitToRange(position);
    }

    public int getDesiredPosition() {
        return desiredPosition;
    }

    public void goToPosition() {
        mWristMotor.set(TalonFXControlMode.Position, desiredPosition);
    }

    public boolean inRange() {
        int encoder = (int) mWristMotor.getSelectedSensorPosition();
        return (encoder > (desiredPosition - Constants.Wrist.threshold)) && (encoder < (desiredPosition + Constants.Wrist.threshold));
    }

    public double GetPosition() {
        return (int) mWristMotor.getSelectedSensorPosition();
     }

     public double GetPositionInches(){
        return  (GetPosition()*(Constants.Wrist.Ratio))/61440;
     }

     public void SetPosition(double position){
        mWristMotor.setSelectedSensorPosition(position);
     }

     public void SetHomePosition(){
        mWristMotor.setSelectedSensorPosition(0);
     }

    private int fitToRange(int position) {
        desiredPosition = Math.min(position, Constants.Wrist.forwardSoftLimit);
        desiredPosition = Math.max(desiredPosition, Constants.Wrist.reverseSoftLimit);
        return desiredPosition;
    }
}