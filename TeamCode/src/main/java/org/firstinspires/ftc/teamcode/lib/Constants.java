package org.firstinspires.ftc.teamcode.lib;


import org.firstinspires.ftc.teamcode.lib.math.InterpolatingDouble;
import org.firstinspires.ftc.teamcode.lib.math.InterpolatingTreeMap;

public class Constants {
    public static class ShootingParams{
        public static double[][] kRPMValues = {
                {13.26, 3300},
                {-15.62, 5500},

        };
        public static double[][] kHoodValues = {
                {13.26, 0.0},
                {0.60,10.0},
                {-7.16,20.0},
                {-13.48,26.0},
                {-15.62,31.0}
        };
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodMap = new InterpolatingTreeMap<>();
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRPMMap = new InterpolatingTreeMap<>();
        static {
            for (double[] pair : kRPMValues) {
                kRPMMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
            }

            for (double[] pair : kHoodValues) {
                kHoodMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
            }
        }
    }

}
