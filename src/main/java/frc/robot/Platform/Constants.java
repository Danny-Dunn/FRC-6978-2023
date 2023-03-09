package frc.robot.Platform;

public class Constants {
    public static class Drive {
        public static double kP = 0.01;
        public static double kI = 0.0003;
        public static double kD = 1;

        public static double peakOutput = 1;

        public static double xMultiplier = 0.3;
        public static double yMultiplier = 1;
        public static double maxX = 1;
        public static double maxY = 1;        
        public static double deadZone = 0.1;

        public static double maxVelocity = 20000;

        public static boolean gearShiftDefaultState = false;

        public static double balancer_yaw_kP = 0.001;
        public static double balancer_yaw_kI = 0.000;
        public static double balancer_yaw_kD = 0.000;

        public static double balancer_pitch_kP = 0.1;
        public static double balancer_pitch_kI = 0.0;
        public static double balancer_pitch_kD = 0.0;
    }
    
    public static class Arm {
        public static double slideStartingPosition = 0;
        public static double slideParkPosition = 0;
        public static double slideMidPosition = 8000;
        public static double slideMaxExtension = 19000;
        public static double slide_kP = 0.0;
        public static double slide_kI = 0.0;
        public static double slide_kD = 0.0;
        
        public static double liftStartingPosition = 0;
        public static double liftLowerPosition = -800000;
        public static double liftUpperPosition = 0;
        //TODO: fix lift PID parameters
        public static double lift_kP = 0.02;
        public static double lift_kI = 0.0;
        public static double lift_kD = 0.0;

        public static double cableStartingPosition = 0;
        public static double cable_kP = 0.01;
        public static double cable_kI = 0.0;
        public static double cable_kD = 0.0;

        public static boolean gripperCloseDefaultState = false;
    }

    public static class DriverControls {
        public static int forwardAxis = 4;
        public static int reverseAxis = 4;


        public static int gearShift = 10;
        public static int balance = 11;
    }

    public static class OperatorControls {
        public static int retractSlide = 5;
        public static int midDeploySlide = 6;
        public static int fullDeploySlide = 8;
    }
}
