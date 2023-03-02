// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class OI {
    public static final class XboxController {
        public static ShuffleboardTab X1_TAB =  Shuffleboard.getTab("Xbox Controller #1");
        public static ShuffleboardTab X2_TAB =  Shuffleboard.getTab("Xbox Controller #2");
        
        public static double X1_RTValue = 0;
        public static double X1_LTValue = 0;

        public static boolean X1_RB = false;
        public static boolean X1_LB = false;

        public static double X1_LJX = 0;
        public static double X1_LJY = 0;
        public static double X1_RJX = 0;
        public static double X1_RJY = 0;

        public static boolean X1_XButton = false;
        public static boolean X1_YButton = false;
        public static boolean X1_AButton = false;
        public static boolean X1_BButton = false;

        public static double X2_RTValue = 0;
        public static double X2_LTValue = 0;

        public static double X2_DPad = -1;

        public static boolean X2_RB = false;
        public static boolean X2_LB = false;

        public static double X2_LJX = 0;
        public static double X2_LJY = 0;
        public static double X2_RJX = 0;
        public static double X2_RJY = 0;

        public static boolean X2_LJS = false;

        public static boolean X2_XButton = false;
        public static boolean X2_YButton = false;
        public static boolean X2_AButton = false;
        public static boolean X2_BButton = false;

        public static GenericEntry X1_RT_Entry = X1_TAB.add("Right Trigger", 0.0).getEntry();
        public static GenericEntry X1_LT_Entry = X1_TAB.add("Left Trigger", 0.0).getEntry();

        public static GenericEntry X1_RB_Entry = X1_TAB.add("Right Bumper", false).getEntry();
        public static GenericEntry X1_LB_Entry = X1_TAB.add("Left Bumper", false).getEntry();

        public static GenericEntry X1_LJX_Entry = X1_TAB.add("Left Joystick X", 0.0).getEntry();
        public static GenericEntry X1_LJY_Entry = X1_TAB.add("Left Joystick Y", 0.0).getEntry();
        public static GenericEntry X1_RJX_Entry = X1_TAB.add("Right Joystick X", 0.0).getEntry();
        public static GenericEntry X1_RJY_Entry = X1_TAB.add("Right Joystick Y", 0.0).getEntry();

        public static GenericEntry X1_XButtonEntry = X1_TAB.add("X Button", false).getEntry();
        public static GenericEntry X1_YButtonEntry = X1_TAB.add("Y Button", false).getEntry();
        public static GenericEntry X1_AButtonEntry = X1_TAB.add("A Button", false).getEntry();
        public static GenericEntry X1_BButtonEntry = X1_TAB.add("B Button", false).getEntry();

        
        public static GenericEntry X2_RT_Entry = X2_TAB.add("Right Trigger", 0.0).getEntry();
        public static GenericEntry X2_LT_Entry = X2_TAB.add("Left Trigger", 0.0).getEntry();

        public static GenericEntry X2_RB_Entry = X2_TAB.add("Right Bumper", false).getEntry();
        public static GenericEntry X2_LB_Entry = X2_TAB.add("Left Bumper", false).getEntry();

        public static GenericEntry X2_LJX_Entry = X2_TAB.add("Left Joystick X", 0.0).getEntry();
        public static GenericEntry X2_LJY_Entry = X2_TAB.add("Left Joystick Y", 0.0).getEntry();
        public static GenericEntry X2_RJX_Entry = X2_TAB.add("Right Joystick X", 0.0).getEntry();
        public static GenericEntry X2_RJY_Entry = X2_TAB.add("Right Joystick Y", 0.0).getEntry();

        public static GenericEntry X2_XButtonEntry = X2_TAB.add("X Button", false).getEntry();
        public static GenericEntry X2_YButtonEntry = X2_TAB.add("Y Button", false).getEntry();
        public static GenericEntry X2_AButtonEntry = X2_TAB.add("A Button", false).getEntry();
        public static GenericEntry X2_BButtonEntry = X2_TAB.add("B Button", false).getEntry();

        public static GenericEntry X2_DPadEntry = X2_TAB.add("DPad", 0).getEntry();

    }
    public static final class Drivetrain  {
        public static ShuffleboardTab Drive_TAB = Shuffleboard.getTab("Drivetrain");

        public static double gyroRate = 0;
        public static double gyroHeading = 0;

        public static double poseX;
        public static double poseY;

        public static GenericEntry GyroRateEntry = Drive_TAB.add("Gyro rate", 0).getEntry();
        public static GenericEntry GyroHeadingEntry = Drive_TAB.add("Gyro Heading", 0).getEntry();
        public static GenericEntry PoseXEntry = Drive_TAB.add("Pose X Meters", 0).getEntry();
        public static GenericEntry PoseYEntry = Drive_TAB.add("Pose Y Meters", 0).getEntry();
    }
    
    public static final class Vision {
        public static ShuffleboardTab Vision_TAB = Shuffleboard.getTab("Vision");

        public static double microYaw = 0;

        public static boolean aprilCamHasTargets = false;

        public static Transform3d distanceToTagX;
        public static Transform3d distanceToTagY;

        public static double pitchToTag= 0;

        public static double PIDOutput = 0;
        public static double error = 0;

        public static GenericEntry MicroYawEntry = Vision_TAB.add("Microsoft Cam Yaw", 0).getEntry();
        public static GenericEntry LimelightTargetsEntry = Vision_TAB.add("Limelight has Targets", false).getEntry();
        public static GenericEntry distanceToTagXEntry = Vision_TAB.add("Distance to Tag X", 0).getEntry();
        public static GenericEntry distanceToTagYEntry = Vision_TAB.add("Distance to Tag Y", 0).getEntry();
        public static GenericEntry PIDoutputEntry = Vision_TAB.add("PID Output", 0).getEntry();
        public static GenericEntry PIDError = Vision_TAB.add("PID Error", 0).getEntry();

    }
    public static final class ElbowPID{
    public static ShuffleboardTab PIDElbow_TAB =  Shuffleboard.getTab("PID Sholder");
        

    public static GenericEntry Kp_Entry = PIDElbow_TAB.add("Elbow kp",1).getEntry();
    public static GenericEntry Ki_Entry = PIDElbow_TAB.add("Elbow ki", 0).getEntry();
    public static GenericEntry Kd_Entry = PIDElbow_TAB.add("Elbow kd", 0).getEntry();
}

}
