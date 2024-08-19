package frc.robot.utils;

public class SwerveUtil {
    public static double remapAngle(double currentAngle, double desiredAngle) {
        if(Math.abs(desiredAngle - currentAngle) > 90){
            double alternateDesiredangle =  (desiredAngle + 180) % 360;
            if(Math.abs(desiredAngle - currentAngle) < Math.abs(alternateDesiredangle - currentAngle)){
                return desiredAngle;
            }
            else{
                return alternateDesiredangle;
            }
        }
        else {
            return desiredAngle;
        }
    }
}
