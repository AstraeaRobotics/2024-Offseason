// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
public class LimelightUtil {
 public static NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable("limelight");
    }

    public static void setPipeline(double pipelineID) {
        getTable().getEntry("pipeline").setNumber(pipelineID); // 4, 7 speaker
    }

    public static double getTx() { // x offset
        return getTable().getEntry("tx").getDouble(0);
    }

    public static double getTy() { // y offset
        return getTable().getEntry("ty").getDouble(0);
    }

    public static double getTa() { // april tag area
        return getTable().getEntry("ta").getDouble(0);
    }
    
    public static int getTagId() {
        double tagId = getTable().getEntry("tid").getDouble(-1);
        return (int) tagId;
    }

    public static double getShooterAngle(double area) {
        return 24.8 + (-0.483 * area) + (11.6 * area * area);
        // 24.8 + -0.483x + 11.6x^2
    }

    public static double[] getBotPose() {
        return getTable().getEntry("pose").getDoubleArray(new double[6]);
    }

    public static double getPoseX() {
        return getBotPose()[0];
    }

    public static double getPoseY() {
        return getBotPose()[1];
    }

    public static double getPoseZ() {
        return getBotPose()[2];
    }

}