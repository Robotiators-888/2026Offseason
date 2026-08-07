package frc.robot;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.util.Units;

public class HoodCorrectnessTest {
    static double ScoreHeight = 8.5; // dummy height in inches

    public static double findoptimalangleOriginalFixed(double distance) {
        double lowestrpm = Double.MAX_VALUE;
        double optimalangle = 0;
        double height = Units.inchesToMeters(ScoreHeight);
        double minexitrange = Units.radiansToDegrees(Math.atan2(2*height,distance));
        double maxexitrange = 85;
        for (int i = (int)Math.ceil(minexitrange); i < maxexitrange; i ++) {
            // Fix: Math.tan(Units.degreesToRadians(i))
            double rad = Units.degreesToRadians(i);
            double exitvelocity = (1/Math.cos(rad))*Math.sqrt((9.8*distance*distance)/(2*(distance*Math.tan(rad)-height)));
            if (exitvelocity < lowestrpm) {
                lowestrpm = exitvelocity;
                optimalangle = i;
            }
        }
        return optimalangle;
    }

    public static double findoptimalangleCalculus(double distance) {
        double height = Units.inchesToMeters(ScoreHeight);
        double optimalAngleRad = Math.atan((height + Math.sqrt(height * height + distance * distance)) / distance);
        return Units.radiansToDegrees(optimalAngleRad);
    }

    @Test
    public void testValues() {
        for (double d = 1.0; d <= 5.0; d += 1.0) {
            System.out.println("Distance: " + d +
                ", Original(fixed): " + findoptimalangleOriginalFixed(d) +
                ", Calculus: " + findoptimalangleCalculus(d));
        }
    }
}
