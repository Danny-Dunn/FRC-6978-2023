package frc.robot.Platform;

public class Globals {
    public static boolean armAutomation = true;
    public static ArmPosition requestedArmPosition = ArmPosition.park;
    public static class ArmConstraints {
        public static boolean liftClawPlexiglassCrash;
        public static boolean liftClawTiltCrash;

        public static boolean cableBumperClearance;
    }
}
