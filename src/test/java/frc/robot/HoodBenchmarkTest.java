package frc.robot;

import org.junit.jupiter.api.Test;
import edu.wpi.first.math.util.Units;

public class HoodBenchmarkTest {
    static double ScoreHeight = 8.5; // dummy height in inches
    static double ShooterDiameter = 4.0;
    static double CompressionValue = 0.5;

    public static double findoptimalangleOriginal(double distance) {
        double lowestrpm = Double.MAX_VALUE;
        double optimalangle = 0;
        double height = Units.inchesToMeters(ScoreHeight);
        double minexitrange = Units.radiansToDegrees(Math.atan2(2*height,distance));
        double maxexitrange = 85;
        for (int i = (int)Math.ceil(minexitrange); i < maxexitrange; i ++) {
            double exitvelocity = (1/Math.cos(Units.degreesToRadians(i)))*Math.sqrt((9.8*distance*distance)/(2*(distance*Math.tan(i)-height)));
            double exitRPM = ((720 / ShooterDiameter)*exitvelocity)/(CompressionValue * Math.PI);
            if (exitRPM < lowestrpm) {
                lowestrpm = exitRPM;
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
    public void testBenchmark() {
        System.out.println("Warmup...");
        for (int i = 0; i < 10000; i++) {
            findoptimalangleOriginal(5.0);
            findoptimalangleCalculus(5.0);
        }

        System.out.println("Benchmarking...");
        long startOrig = System.nanoTime();
        for (int i = 0; i < 100000; i++) {
            findoptimalangleOriginal(5.0);
        }
        long endOrig = System.nanoTime();

        long startCalc = System.nanoTime();
        for (int i = 0; i < 100000; i++) {
            findoptimalangleCalculus(5.0);
        }
        long endCalc = System.nanoTime();

        double origMs = (endOrig - startOrig) / 1e6;
        double calcMs = (endCalc - startCalc) / 1e6;

        System.out.println("Original time for 100,000 iterations: " + origMs + " ms");
        System.out.println("Calculus time for 100,000 iterations: " + calcMs + " ms");
        System.out.println("Speedup: " + (origMs / calcMs) + "x");
    }
}
