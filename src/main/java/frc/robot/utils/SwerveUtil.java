package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveUtil {
    public static double[] optimizeModule(double currentAngle, double desiredAngle, double speed) {
        double[] optimizedModule = new double[2];
        double angleDifference = desiredAngle - currentAngle;
        optimizedModule[0] = desiredAngle;
        optimizedModule[1] = -speed;
        
        // Optimizes the desired angle so the module never turns more than 90 degrees
        if(Math.abs(angleDifference) > 90 && Math.abs(angleDifference) < 270) {
            optimizedModule[0] = (desiredAngle + 180)  % 360;
            optimizedModule[1] = speed;
        }

        // Drive speed of module proportional to its closeness to desired angle to prevent slippage
        // optimizedModule[1] *= Math.abs(Math.cos(Math.abs(currentAngle - optimizedModule[0])));

        return optimizedModule;
    }

}
