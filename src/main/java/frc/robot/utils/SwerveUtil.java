package frc.robot.utils;

public class SwerveUtil {
    public static double remapAngle(double currentAngle, double desiredAngle) {
        double angleDifference = Math.abs(currentAngle - desiredAngle);
        return angleDifference > 90 ? ((angleDifference + 180) % 360) : angleDifference;
    }

    public static double remapSpeed(double desiredAngle, double speed) {
        return desiredAngle > 180 && desiredAngle < 360 ? -speed : speed;
    }
}
