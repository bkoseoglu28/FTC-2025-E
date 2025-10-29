package org.firstinspires.ftc.teamcode.lib;


import org.firstinspires.ftc.teamcode.lib.math.InterpolatingDouble;
import org.firstinspires.ftc.teamcode.lib.math.InterpolatingTreeMap;

public class Constants {
    public static class ShootingParams{
        public static double[][] kRPMValues = {
                {9.78, 3000},
                {-15.39, 4000},

        };
        public static double[][] kHoodValues = {
                {9.78,11},//15
                {-6.30,26},
                {-13.08,15},
                {-15.39,11}
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
