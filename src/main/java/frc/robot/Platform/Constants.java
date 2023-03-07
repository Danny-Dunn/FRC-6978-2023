package frc.robot.Platform;

public class Constants {
    static class Drive {
        static double kP = 0.01;
        static double kI = 0.0003;
        static double kD = 1;

        static double peakOutput = 1;

        static double xMultiplier = 1/3;
        static double yMultiplier = 1;
        static double deadZone = 0.1;
    }
    
    static class Arm {
        static double slideMaxExtension = 19000;
        static double slide_kP = 0.0;
        static double slide_kI = 0.0;
        static double slide_kD = 0.0;
        
        static double liftStartingPosition = 0;
        //TODO: fix lift PID parameters
        static double lift_kP = 0.02;
        static double lift_kI = 0.0;
        static double lift_kD = 0.0;
    }
}
