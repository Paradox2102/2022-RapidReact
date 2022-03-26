
package frc.robot;

import java.io.File;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public). Do not put anything functional in this class.
 *
 * <p>
 * It is advised toally import this class (or one of its inner classes) wherever
 * the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    // Talons
    public static Constants c = null;

    public static Constants getInstance() {
        if (c == null)
        {
            File f = new File("/home/lvuser/Competition");
    
            // Checking if the specified file exists or not
            if (f.exists())
            {
                c = new ConstantsCompetition();
                SmartDashboard.putString("RobotType", "competition"); 
            }
            else {
                c = new Constants(); 
                SmartDashboard.putString("RobotType", "practice"); 
            }
        }

        return c;
    }
    // Drive FX
    public int k_driveRight = 2;
    public int k_driveRightFollower = 12;
    public int k_driveLeft = 5;
    public int k_driveLeftFollower = 0;
    // Shooter FX
    public int k_shooter = 14;
    public int k_shooterFollower = 15;
    public int k_backWheel = 6;

    // Climber FX
    public int k_climber = 3;
    public int k_climberFollower = 4;
    public int k_rotaterPiston = 0;
    public int k_breakerPiston = 2;
    // public int k_ratchet = 0;
    // Scotty (throat) FX
    public int k_scotty = 13;
    // Intake (spin) SRX
    public int k_intake = 1;

    // Sensors
    // Beambrake
    public int k_scotClose = 0;
    public int k_scotMid = 1;
    public int k_scotFar = 2;

    public int k_magneticLeft;
    public int k_magneticRight;
    public int k_switchLeft;
    public int k_switchRight;

    // Pneumatics
    // Climber
    public int k_piston = 0;
    public int k_claw = 1;
    // Intake
    public int k_deploy = 2;
    // Hood
    public int k_hood1 = 5;
    public int k_hood2 = 6;
    // Servo
    public int k_servo = 0;

    // Conversions
    public double k_feetPerTick = 12.58 / 188529;
}

class ConstantsCompetition extends Constants {
    // Talons
    ConstantsCompetition() {
        // Drive FX
        k_driveRight = 2;
        k_driveRightFollower = 12;
        k_driveLeft = 5;
        k_driveLeftFollower = 0;
        // Shooter FX
        k_shooter = 14;
        k_shooterFollower = 15;
        k_backWheel = 6;
 
        // Climber FX
        k_climber = 3;
        k_climberFollower = 4;
        // Scotty (throat) FX
        k_scotty = 13;
        // Intake (spin) SRX
        k_intake = 1;

    // Sensors
        // Beambrake
        k_scotClose = 0;
        k_scotMid = 1;
        k_scotFar = 2;
        // Magnetic
        k_magneticLeft = 3;
        k_magneticRight = 9;
        // Physical
        k_switchLeft = 8;
        k_switchRight = 7;

    // Pneumatics
        // Climber
        k_rotaterPiston = 0;
        k_breakerPiston = 2;
        // Intake
        k_deploy = 1;
        // Hood
        
    // Servos
        k_hood1 = 5;
        k_hood2 = 6;
        k_servo = 9;

    // Conversions
        k_feetPerTick = 12.58 / 188529;
    }
}
