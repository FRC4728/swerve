package frc.robot;

public final class Constants {

    /**
    * These are the constants which are used to map the hardware and define the
    * working bahavior of the hardware which isn't used by any individual
    * subsystem.
    */      
    public static final class HARDWARE {
        public static final int SWERVE_POD_1_QUAD_A_DIO_CHANNEL = 0;            // DIO channel used for the quadrature encoder signal A
        public static final int SWERVE_POD_1_QUAD_B_DIO_CHANNEL = 1;            // DIO channel used for the quadrature encoder signal B
        public static final int SWERVE_POD_1_PWM_DIO_CHANNEL = 2;               // DIO channel used for the absolute encoder
    }

    /**
    * These are the constants which are used to map the hardware and define the
    * working bahavior of the driver and operator controls.
    * @see {@link frc.robot.RobotContainer#ConfigureButtonBindings}
    */      
    public static final class DRIVER {
        public static final int JOYSTICK_TURN = 0;                              // Turn joystick port number
        public static final int JOYSTICK_THROTTLE = 1;                          // Throttle joystick port number
        public static final int DRIVER_BUTTON_BOARD = 2;                        // Drive button controller port number
    }

    /**
    * These are the constants which are used to map the hardware and define the
    * working bahavior of the the drivetrain subsystem.
    * @see {@link frc.robot.subsystems.Drivetrain}
    */
    public static final class DRIVETRAIN {
        public static final double WHEEL_DIAMETER_INCH = 4.0;                   // Average wheel diameter


        // public static final int LEFT_MASTER_ID = 15;                            // Motor controller CAN ID AND PDP Port number
        // public static final int LEFT_FOLLOWER_1_ID = 14;                        // Motor controller CAN ID AND PDP Port number
        // public static final int LEFT_FOLLOWER_2_ID = 13;                        // Motor controller CAN ID AND PDP Port number
        // public static final int RIGHT_MASTER_ID = 0;                            // Motor controller CAN ID AND PDP Port number
        // public static final int RIGHT_FOLLOWER_1_ID = 1;                        // Motor controller CAN ID AND PDP Port number
        // public static final int RIGHT_FOLLOWER_2_ID = 2;                        // Motor controller CAN ID AND PDP Port number
        // public static final double ENCODER_GEARING = (60.0/24.0)*(36.0/12.0);   // Gearing between encoder and output shaft (output/input)
        // public static final double ENCODER_PPR = 4096.0;                        // Encoder pulses per revolution        
        // public static final double WHEEL_DIAMETER_METERS = 0.157;               // Wheel diamter 6-3/16"
        // public static final double kS = 1.45;
        // public static final double kV = 1.82;
        // public static final double kA = 0.517;
        // public static final double kTRACK_WIDTH_METERS = 0.647;
        // public static final  double MAX_SPEED_METERS_PER_SECOND = 1.5;
        // public static final  double MAX_ACCELERATIOIN_METERS_PER_SECOND_SECOND = 1.5;
        // public static final  double MAX_VOLTAGE = 10.0;
        // public static final int LOW_GEAR_SOLENOID_ID = 0;                       // PCM port number for low gear shifting
        // public static final int HIGH_GEAR_SOLENOID_ID = 1;                      // PCM port number for high gear shifting
        // public static final boolean VISION_THREADED = false;                    // Vision threading flag 
        // public static final double VISION_SEARCH_TIMEOUT_S = 3.0;               // Vision search (look for target) timeout threshold
        // public static final double VISION_SEEK_TIMEOUT_S = 2.0;                 // Vision seek (get on target) timeout threshold
        // public static final int VISION_SEEK_RETRY_LIMIT = 3;                    // Vision seek retry limit threshold
        // public static final double VISION_TURN_PID_P = 2.98 / 29.8;             // Vision turning proportional gain
        // public static final double VISION_TURN_PID_I = 0.0;                     // Vision turning command intgral gain
        // public static final double VISION_TURN_PID_D = 0.0;                     // Vision turning derivative gain
        // public static final double VISION_TURN_PID_F = 0.0;                     // Vision turning feed-forward
        // public static final double VISION_DISTANCE_PID_P = 1.0;                 // Vision driving proportional gain
        // public static final double VISION_DISTANCE_PID_I = 0.0;                 // Vision driving intgral gain
        // public static final double VISION_DISTANCE_PID_D = 0.0;                 // Vision driving derivative gain
        // public static final double VISION_DISTANCE_PID_F = 0.0;                 // Vision drivingd feed-forward
        // public static final double VISION_ON_TARGET_TURN_THRESHOLD_DEG = 1.5;   // Vision turning acceptable error threshold
        // public static final double VISION_ON_TARGET_DISTANCE_THRESHOLD_FT = 0.5;// Vision distance acceptable error threshold
        // public static final DistanceEstimator_t VISION_DISTANCE_ESTIMATOR = DistanceEstimator_t.BoundingBox; // Vision distance estimator calculation
        // public static final double VISION_TARGET_WIDTH_FT = 21.5 / 12.0;        // Vision known target width in feet (bounding box distance estimator)
        // public static final double VISION_FOCAL_LENGTH_FT = (67.0*7.0)/VISION_TARGET_WIDTH_FT;// Vision focal length = (measured_target_pixels * known_distance_to_taget) / known_target_width (bounding box distance estimator)
        // public static final double VISION_FLOOR_TO_TARGET_FT = 26.0 / 12.0;     // Vision known floor-to-target (fixed angle distance estimator)
        // public static final double VISION_FLOOR_TO_LIMELIGHT_FT = 1.0;          // Vision known floor-to-limelight (fixed angle distance estimator)
        // public static final double VISION_LIMELIGHT_MOUNT_ANGLE_DEG = 0.0;      // Vision known limelight mounting angle (fixed angle distance estimator)
    }

}
