package org.WaialuaRobotics359.robot.subsystems;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.WaialuaRobotics359.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;


public class LEDs extends SubsystemBase {
    private final CANdle candle;

    private final StrobeAnimation PurpleStrobe;
    private final StrobeAnimation YellowStrobe;

    private final StrobeAnimation WarningFlash;

    private final LarsonAnimation BlueLarson;
    private final LarsonAnimation RedLarson;

    public static Boolean hasObject = false; 
    public static Boolean autoStartPose = false; 
    public static boolean reportWarning = true; 

    public static enum State {off,purple,yellow,Blue,Red,Green}
    public State state = State.off;

    /*Logging*/
    private DataLog logger;
    /*LED logs*/
    private StringLogEntry LEDState;
    private BooleanLogEntry LEDHasObject;
    private BooleanLogEntry LEDAutoStartPose;
    private BooleanLogEntry LEDReportWarning;


    public LEDs() {
        candle = new CANdle(Constants.LEDs.CANdleID, "rio");

        PurpleStrobe = new StrobeAnimation(147, 0, 255, 0, .2, Constants.LEDs.LEDCount, 0);
        YellowStrobe = new StrobeAnimation(255, 64, 0, 0, .2, Constants.LEDs.LEDCount, 0);

        BlueLarson = new LarsonAnimation(0, 0, 255, 0, .7, Constants.LEDs.LEDCount, BounceMode.Center, 15, 0);
        RedLarson = new LarsonAnimation(255, 0, 0, 0, .7, Constants.LEDs.LEDCount, BounceMode.Center, 15, 0);

        WarningFlash = new StrobeAnimation(255, 64, 0, 0, .2, Constants.LEDs.LEDCount, 0);

        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 1; 

        candle.configAllSettings(config);

        /*Logging*/
        logger = DataLogManager.getLog();
        LEDState = new StringLogEntry(logger, "LEDs/LEDState");
        LEDHasObject = new BooleanLogEntry(logger, "LEDs/LEDHasObject");
        LEDAutoStartPose = new BooleanLogEntry(logger, "LEDs/LEDAutoStartPose");
        LEDReportWarning = new BooleanLogEntry(logger, "LEDs/LEDReportWarning");
    }

    @Override
    public void periodic(){
        switch(state){
            case off:
                candle.setLEDs(0,0,0);
                break;
            case purple:
                if (hasObject){
                    candle.animate(PurpleStrobe, 1);
                }else{
                    candle.setLEDs(147,0,255);
                    candle.clearAnimation(1);
                }
                break;
            case yellow:
                if (hasObject){
                    candle.animate(YellowStrobe, 1);
                }else{
                    candle.setLEDs(255,65,0);
                    candle.clearAnimation(1);
                }
                break;
            case Blue:
                if (reportWarning){
                    candle.animate(WarningFlash, 1);
                }
                else if(autoStartPose && !reportWarning){
                    candle.animate(BlueLarson, 1);
                }else{
                    candle.setLEDs(0,0,255);
                    candle.clearAnimation(1);
                }
                break;
            case Red:
                if (reportWarning){
                    candle.animate(WarningFlash, 1);
                }
                else if(autoStartPose && !reportWarning){
                    candle.animate(RedLarson, 1);
                }else{
                    candle.setLEDs(255,0,0);
                    candle.clearAnimation(1);
                }
                break;
            case Green:
                candle.clearAnimation(1);
                candle.setLEDs(0, 155, 0);
            default:
                candle.setLEDs(0,0,0);
                break;
        }
        LogData(RobotController.getFPGATime());
    }

    public void setLEDs(int i, int j, int k) {
    }

    public void setLEDAliance (){
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue){
            state = State.Blue;
        } else {
            state = State.Red;
        }
    }

    private void LogData(long time){
        LEDState.append(state.toString(), time);
        LEDHasObject.append(hasObject, time);
        LEDAutoStartPose.append(autoStartPose, time);
        LEDReportWarning.append(reportWarning, time);
    }

}
   