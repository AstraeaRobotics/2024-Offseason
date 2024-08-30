package frc.robot.utils;

public class SwerveUtil {
    public static double remapAngle(double currentAngle, double desiredAngle) {
        // double angleDifference = Math.abs(currentAngle - desiredAngle);
        // return Math.abs(angleDifference) > 90 ? ((desiredAngle + 180) % 360) : desiredAngle;
        double normalizedAngleDifference = ((desiredAngle - currentAngle) + 180) % 360 - 180;
        return Math.abs(normalizedAngleDifference) > 90 ? ((normalizedAngleDifference + 180) % 360) : normalizedAngleDifference;
       
    }

    public static double remapSpeed(double desiredAngle, double speed) {
        return desiredAngle < 0 && desiredAngle > -180 ? -speed : speed;
    }
}
