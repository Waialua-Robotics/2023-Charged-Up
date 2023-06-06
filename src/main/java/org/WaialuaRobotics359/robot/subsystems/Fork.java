package org.WaialuaRobotics359.robot.subsystems;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.Robot;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.hardware.TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Fork extends SubsystemBase {
    private TalonFX mForkMotor;
    Servo leftServo;
    Servo rightServo;

    public TalonFXConfigurator ForkFXConfigurator = mForkMotor.getConfigurator();

    /*Logging*/
    private DataLog logger;
    /*elevator logs*/
    private DoubleLogEntry forkMotorCurrent;
    private DoubleLogEntry forkMotorTemperature;
    private DoubleLogEntry forkLeftServoPosition;
    private DoubleLogEntry forkRightServoPosition;

    public Fork() {
        mForkMotor = new TalonFX(Constants.Fork.forkMotorID);

        mForkMotor.getConfigurator().apply(Robot.ctreConfigs.forkFXConfig);
        mForkMotor.setInverted(Constants.Fork.forkMotorInvert);
        mForkMotor.setNeutralMode(Constants.Fork.forkNeutralMode);

        leftServo = new Servo(0);
        rightServo = new Servo(1);

        leftServo.set(1);
        rightServo.set(0);

        /*Logging*/
        logger = DataLogManager.getLog();
        forkMotorCurrent = new DoubleLogEntry(logger, "fork/motorCurrent");
        forkMotorTemperature = new DoubleLogEntry(logger, "fork/motorTemperature");
        forkLeftServoPosition = new DoubleLogEntry(logger, "fork/leftServoPosition");
        forkRightServoPosition = new DoubleLogEntry(logger, "fork/rightServoPosition");
    }

    public void openLatch() {
        leftServo.set(0);
        rightServo.set(1);
    }

    public void closeLatch() {
        leftServo.set(1);
        rightServo.set(0);
    }

    public void SetPrecentOut(double percent){
        mForkMotor.set(TalonFXControlMode.PercentOutput, percent);
    }

    public void Stop(){
        SetPrecentOut(0);
    }

    private void LogData(long time){
        forkMotorCurrent.append(mForkMotor.getSupplyCurrent().getValue(), time);
        forkMotorTemperature.append(mForkMotor.getDeviceTemp().getValue(), time);
        forkLeftServoPosition.append(leftServo.get(), time);
        forkRightServoPosition.append(rightServo.get(), time);
    }

    public void periodic(){
        LogData(RobotController.getFPGATime());
    }
}

