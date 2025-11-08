package org.firstinspires.ftc.teamcode.lib;


import org.firstinspires.ftc.teamcode.lib.math.InterpolatingDouble;
import org.firstinspires.ftc.teamcode.lib.math.InterpolatingTreeMap;

public class ShooterConstants {
    public static class ShootingParams{
//        public static double[][] kRPMValues = {
//                {9.78, 3000},
//                {-15.39, 4300},
//
//        };
//        public static double[][] kHoodValues = {
//                {9.78,9},//15
//                {-6.30,26},
//                {-13.08,15},
//                {-15.39,11}
//        };
        public static double[][] kRPMValues = {
                {14.15, 3900},
                {-10.66, 4700},
                {-16.01, 5500}
        };
        public static double[][] kHoodValues = {
                {14.15, 0},
                {-6.75, 19},
                {-10.66, 24},
                {-14.48, 28},
                {-16.40, 32}
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
