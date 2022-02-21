
package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Talons
        // Drive FX
        public static int k_driveRight = 2;
        public static int k_driveRightFollower = 12;
        public static int k_driveLeft = 5;
        public static int k_driveLeftFollower = 0;
        // Shooter FX
        public static int k_shooter = 14;
        public static int k_shooterFollower = 15;
        // Climber FX
        public static int k_climber = 4;
        // Scotty (throat) FX
        public static int k_scotty = 13;
        // Intake (spin) SRX
        public static int k_intake = 1;

    // Sensors
        // Beambrake
        public static int k_scotClose = 0;
        public static int k_scotMid = 1;
        public static int k_scotFar = 2;
        
    // Pneumatics
        // Climber
        public static int k_piston = 0;
        public static int k_claw = 1;
        // Intake
        public static int k_deploy = 2;
        // Hood
        public static int k_hood = 3;
    // Servo
    public static int k_servo = 0;
}
