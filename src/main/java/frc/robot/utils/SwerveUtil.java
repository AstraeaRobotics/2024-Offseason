package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveUtil {
    public static double remapAngle(double currentAngle, double desiredAngle) {
        double angleDifference = desiredAngle - currentAngle;
        return Math.abs(angleDifference) > 90 && Math.abs(angleDifference) < 270 ? (desiredAngle + 180) % 360 : desiredAngle;
    }


    public static double remapSpeed(double displacement, double speed) {
        return Math.abs(displacement) > 90 && Math.abs(displacement) < 270 ? speed : -speed;
    }

}
