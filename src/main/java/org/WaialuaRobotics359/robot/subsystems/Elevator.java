package org.WaialuaRobotics359.robot.subsystems;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private TalonFX mElevatorMotorL;
    public static TalonFX mElevatorMotorR;



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

    public void setDesiredPosition(int position) {
        desiredPosition = fitToRange(position);
    }

    public int getDesiredPosition() {
        return desiredPosition;
    }

    public void goToPosition() {
        mElevatorMotorR.set(TalonFXControlMode.Position, desiredPosition);
    }

    public boolean inRange() {
        int encoder = (int) mElevatorMotorR.getSelectedSensorPosition();
        return (encoder > (desiredPosition - Constants.Elevator.threshold)) && (encoder < (desiredPosition + Constants.Elevator.threshold));
    }

    public double GetPosition() {
        return mElevatorMotorR.getSelectedSensorPosition();
     }

     public double GetPositionInches(){
       // return  (GetPosition()*(Constants.Elevator.Ratio))/61440;
       return GetPosition();
     }

     public void SetPosition(double position){
        mElevatorMotorR.setSelectedSensorPosition(position);
     }

     public void SetHomePosition(){
        mElevatorMotorR.setSelectedSensorPosition(0);
     }
/* 
     public static void SetElevatorPositionInches (double desired){
        double position = (desired)/(Constants.Elevator.Ratio)*2048;
        SetElevatorPositionInches(position);
        Elevator.mElevatorMotorR.set(TalonFXControlMode.Position, position);
     }
     */
    /*  public static void SetElevatorInches(double desired){
        double position = (desired*61440)/(Constants.Elevator.Ratio);
        SetElevatorInches(position);
        mElevatorMotorR.set(TalonFXControlMode.Position, position);
     }

     public void LowElevatorPositionInches(){
        SetElevatorInches(30);
     }
*/
    private int fitToRange(int position) {
        desiredPosition = Math.min(position, Constants.Elevator.forwardSoftLimit);
        desiredPosition = Math.max(desiredPosition, Constants.Elevator.reverseSoftLimit);
        return desiredPosition;
    }

}
