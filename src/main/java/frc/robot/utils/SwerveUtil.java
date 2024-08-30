package frc.robot.utils;

public class SwerveUtil {
    public static double remapAngle(double currentAngle, double desiredAngle) {
        // double angleDifference = Math.abs(currentAngle - desiredAngle);
        double angleDifference = currentAngle - desiredAngle;
        if(angleDifference > 180) {
            desiredAngle = (desiredAngle - 360) % 360;
        }
        else if (angleDifference < -180) {
            desiredAngle = (desiredAngle + 360) % 360;
        }
        angleDifference = currentAngle - desiredAngle;

        return Math.abs(angleDifference) > 90 ? ((desiredAngle + 180) % 360) : desiredAngle;
    }

    public static double remapSpeed(double desiredAngle, double speed) {
        return desiredAngle < 0 && desiredAngle > -180 ? -speed : speed;
    }
}
