
package frc.robot;


public final class Constants {
    
    // Drive Constants
    public static final class DriveConstants {
        // Left Drive
        public static final int kFrontLeftMotor = 1;
        public static final int kBackLeftMotor = 2;
        // Right Drive
        public static final int kFrontRightMotor = 3;
        public static final int kBackRightMotor = 4;
    }

    // OI Constants
    public static final class OIConstants {
        // Controllers
        public static final int kDriveJoyStick = 0;
        public static final int kActuatorJoyStick = 1;
    }

    // Actuator Constants
    public static final class ActuatorConstants {
        // Turret
        public static final int kTurretMotor = 5;
        // Indexer
        public static final int kIndexerFront = 6;
        
        // Fly Wheel
        public static final int kLeftFlyWheel = 7;
        public static final int kRightFlyWheel = 8;
        // Climber
        public static final int kClimbMotor = 10;
        // Intake (PWM 0)
        public static final int kIntakeMotor = 0;
        // Latch (PWM 1)
        public static final int kLatchMotor = 1;
        // Lights (PWM 2)
        public static final int klights = 2;
    }

    //Sensors Inputs Ball intake and thru the indexer
    public static final class SensorContants {
        // Ready to index from ball intake
        public static final int kSensorInTake = 0;
        // Index location 1
        public static final int kSensorIndex1 = 1;
        // Index location 2
        public static final int kSensorIndex2 = 2;
        // Index location 3
        public static final int kSensorIndex3 = 3;
        // Index location 4
        public static final int kSensorIndex4 = 4;
        // Index location 5
        public static final int kSensorIndex5 = 5;

        public static final int kClimberExtend = 8;

        public static final int kClimberLower = 9;

        public static final int kTurretHome = 7;
    }

    public static final class EncoderConstants{

        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from
         * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
         * configuration.
         */
	    public static final int kSlotIdx = 0;

        /*
        * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
        * now we just want the primary one.
        */
	    public static final int kPIDLoopIdx = 0;

        /*
        * set to zero to skip waiting for confirmation, set to nonzero to wait and
        * report to DS if action fails.
        */
	    public static final int kTimeoutMs = 10;

	    /* Current threshold to trigger current limit */
	    static final int kPeakCurrentAmps = 42;
    
	    /* Duration after current exceed Peak Current to trigger current limit */
	    static final int kPeakTimeMs = 10;
 
	    /* Current to mantain once current limit has been triggered */
	    static final int kContinCurrentAmps = 40;
 
        /**
         * Timeout value generally used in parameter configs
        * Non-zero to block the config until success, zero to skip checking 
        */
	    static final int  CurrentLimitingkTimeoutMs= 30;
    }
}