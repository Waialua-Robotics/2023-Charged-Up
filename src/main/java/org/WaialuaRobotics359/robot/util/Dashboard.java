package org.WaialuaRobotics359.robot.util;

import org.WaialuaRobotics359.robot.Constants;
import org.WaialuaRobotics359.robot.RobotContainer;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Dashboard {
    public Dashboard(RobotContainer container) {
        ShuffleboardTab tab = Shuffleboard.getTab("Subsystem");

        /*tab.addNumber("elevator Height", () -> container.getElevator().GetPositionInches()).withSize(2, 2)
                .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min" , 0 , "max" , 49)); */
        tab.addNumber("Wrist value", () -> container.getWrist().GetPosition());
        tab.addNumber("Slide value", () -> container.getSlide().GetPosition());
        tab.addNumber("Elevator value", () -> container.getElevator().GetPosition());
        tab.addNumber("Wrist desired", () -> container.getWrist().getDesiredPosition());
        tab.addNumber("Slide desired", () -> container.getSlide().getDesiredPosition());
        tab.addNumber("Elevator desired", () -> container.getElevator().getDesiredPosition());
        tab.addBoolean("isCube", () -> RobotContainer.isCube);
        tab.addBoolean("competition mode", ()-> Constants.isCompetitionRobot);
        tab.addBoolean("DriveSlowMode", () -> container.getSwerve().slowMode);
        tab.addNumber("SlideCurrent", () -> container.getSlide().getCurrent());
        tab.addNumber("intakeCurrent", ()-> container.getIntake().getCurrent());
        tab.addNumber("velocityChangeWrist",()-> container.getWrist().GetVelocity());
        tab.addNumber("getCurrentWrist",()-> container.getWrist().getCurrent());
        tab.addNumber("velocityChangeElevator",()-> container.getElevator().GetVelocity());
        tab.addNumber("getCurrentelevator",()-> container.getElevator().getCurrent());
        tab.addNumber("pitch", ()-> container.getSwerve().GetGyroPitch());
        //tab.addNumber("posex", ()-> container.getSwerve().getPose().getX());
        //tab.addNumber("posey", ()-> container.getSwerve().getPose().getY());
        tab.addNumber("Distace to pull",()-> container.getLimelight().getDistance());
        tab.addNumber("tx", ()-> container.getLimelight().getTX());
        tab.addNumber("ty", ()-> container.getLimelight().getTY());
        tab.addBoolean("LimitSwitch", ()-> container.getElevator().getSwitch());
        tab.addNumber("yaw360", ()-> container.getSwerve().getYaw360());
        tab.addNumber("Pitch", ()-> container.getSwerve().GetGyroPitch());
        tab.addBoolean("HasSwitched", ()-> container.getElevator().HasSwitched);
        

        //tab.addCamera("LimeLight", "Limelight", "http://10.3.59.11:5800");



        /*tab.add("Autonomous Mode", container.getAutonomousChooser().getModeChooser()).withSize(2, 1).withPosition(2, 0);
        tab.add("Climb Mode", container.getClimbChooser().getClimbChooser()).withSize(2, 1).withPosition(0, 2);
        tab.addCamera("Camera", "Camera", "http://limelight.local:5800", "http://10.29.10.11:5800").withSize(3, 3)
                .withPosition(4, 0);*/
    }
    
}
