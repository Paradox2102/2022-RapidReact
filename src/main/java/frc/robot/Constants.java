
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

    // Climber FX
    public int k_climber = 4;
    // Scotty (throat) FX
    public int k_scotty = 13;
    // Intake (spin) SRX
    public int k_intake = 1;

    // Sensors
    // Beambrake
    public int k_scotClose = 0;
    public int k_scotMid = 1;
    public int k_scotFar = 2;

    // Pneumatics
    // Climber
    public int k_piston = 0;
    public int k_claw = 1;
    // Intake
    public int k_deploy = 2;
    // Hood
    public int k_hood = 3;
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
 
        // Climber FX
        k_climber = 4;
        // Scotty (throat) FX
        k_scotty = 13;
        // Intake (spin) SRX
        k_intake = 1;

    // Sensors
        // Beambrake
        k_scotClose = 0;
        k_scotMid = 1;
        k_scotFar = 2;

    // Pneumatics
        // Climber
        k_piston = 2;
        k_claw = 0;
        // Intake
        k_deploy = 1;
        // Hood
        k_hood = 3;
    // Servo
        k_servo = 9;

    // Conversions
        k_feetPerTick = 12.58 / 188529;
    }
}
