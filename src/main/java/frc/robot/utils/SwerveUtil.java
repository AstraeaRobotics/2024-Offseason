package frc.robot.utils;

public class SwerveUtil {
    public static double remapAngle(double currentAngle, double desiredAngle) {
        double angleDifference = currentAngle - desiredAngle;
        // return Math.abs(angleDifference) > 90 ? ((desiredAngle + 180) % 360) : desiredAngle;
        
        if (Math.abs(angleDifference) > 90) {
            angleDifference = -(180 - Math.abs(angleDifference)) * Math.signum(angleDifference);
        }

        return angleDifference;
    }

    public static double remapSpeed(double desiredAngle, double speed) {
        return desiredAngle < 0 && desiredAngle > -180 ? -speed : speed;
    }
}
