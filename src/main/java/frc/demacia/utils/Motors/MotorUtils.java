package frc.demacia.utils.Motors;

import edu.wpi.first.math.MathUtil;

public class MotorUtils {
    public static double getPositionForAngle(double position, double angle, boolean isRadians) {
        double pi = isRadians ? Math.PI : 180.0;
        double twoPi = 2.0 * pi;
        
        double currentAngle = MathUtil.inputModulus(position, -pi, pi);
        double diff = angle - currentAngle;
        
        if (diff > pi) {
            diff -= twoPi;
        } else if (diff < -pi) {
            diff += twoPi;
        }
        
        return position + diff;
    }
}