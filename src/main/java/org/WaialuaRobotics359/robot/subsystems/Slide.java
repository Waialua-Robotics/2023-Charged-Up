package org.WaialuaRobotics359.robot.subsystems;

import javax.swing.text.Position;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Slide extends SubsystemBase{
    private TalonFX mSlideMotor;
    private int desiredPosition = 0;

    public Slide () {
        mSlideMotor = new TalonFX(Constants.Slide.slideMotorID);

        mSlideMotor.configFactoryDefault();
        mSlideMotor.configAllSettings(Robot.ctreConfigs.slideFXConfig);
        mSlideMotor.setInverted(Constants.Slide.slideMotorInvert);
        mSlideMotor.setNeutralMode(Constants.Slide.slideNeutralMode);
        mSlideMotor.setSelectedSensorPosition(0);
    }

    public void setDesiredPosition(int position) {
        desiredPosition = fitToRange(position);
    }

    public int getDesiredPosition() {
        return desiredPosition;
    }

    public void goToPosition() {
        mSlideMotor.set(TalonFXControlMode.Position, desiredPosition);
    }

    public boolean inRange() {
        int encoder = (int) mSlideMotor.getSelectedSensorPosition();
        return (encoder > (desiredPosition - Constants.Slide.threshold)) && (encoder < (desiredPosition + Constants.Slide.threshold));
    }

    public int GetPosition() {
        return (int) mSlideMotor.getSelectedSensorPosition();
    }

    public double GetPositionInches(){
        return  (GetPosition()*(Constants.Slide.Ratio))/61440;
    }

    public void SetPrecentOut(double percent){
        mSlideMotor.set(TalonFXControlMode.PercentOutput, percent);
    }

    public void Stop(){
        SetPrecentOut(0);
    }

    public void AutoZero() {
        final double ZeroCurrent = .1;
        if (mSlideMotor.getStatorCurrent() > ZeroCurrent){
            SetHomePosition();
            return;
        }else{
        mSlideMotor.set(TalonFXControlMode.PercentOutput, -.1);
        }   
    } 

    public double getCurrent(){
        return mSlideMotor.getStatorCurrent();
    }



    public void SetPosition(double position){
        mSlideMotor.setSelectedSensorPosition(position);
    }

    public void SetHomePosition(){
        mSlideMotor.setSelectedSensorPosition(0);
    }

    private int fitToRange(int position) {
        desiredPosition = Math.min(position, Constants.Slide.forwardSoftLimit);
        desiredPosition = Math.max(desiredPosition, Constants.Slide.reverseSoftLimit);
        return desiredPosition;
    }
}
