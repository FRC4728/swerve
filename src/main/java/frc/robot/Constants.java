package frc.robot;

public final class Constants {


    /**
     * The physical hardware connections on the robot.
     */
    public static final class HARDWARE {
        public static final int FRONT_LEFT_QUAD_A_DIO_CHANNEL = 9;              // DIO channel used for the quadrature encoder signal A (yellow)
        public static final int FRONT_LEFT_QUAD_B_DIO_CHANNEL = 10;             // DIO channel used for the quadrature encoder signal B (green) (on MXP)
        public static final int FRONT_LEFT_PWM_DIO_CHANNEL = 11;                // DIO channel used for the absolute encoder (blue) (on MXP)
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 12;                 // Motor controller CAN ID AND PDP Port number
        public static final int FRONT_LEFT_TURN_MOTOR_ID = 13;                  // Motor controller CAN ID AND PDP Port number

        public static final int REAR_LEFT_QUAD_A_DIO_CHANNEL = 3;               // DIO channel used for the quadrature encoder signal A (yellow)
        public static final int REAR_LEFT_QUAD_B_DIO_CHANNEL = 4;               // DIO channel used for the quadrature encoder signal B (green)
        public static final int REAR_LEFT_PWM_DIO_CHANNEL = 5;                  // DIO channel used for the absolute encoder (blue)
        public static final int REAR_LEFT_DRIVE_MOTOR_ID = 15;                  // Motor controller CAN ID AND PDP Port number
        public static final int REAR_LEFT_TURN_MOTOR_ID = 14;                   // Motor controller CAN ID AND PDP Port number

        public static final int FRONT_RIGHT_QUAD_A_DIO_CHANNEL = 6;             // DIO channel used for the quadrature encoder signal A (yellow)
        public static final int FRONT_RIGHT_QUAD_B_DIO_CHANNEL = 7;             // DIO channel used for the quadrature encoder signal B (green)
        public static final int FRONT_RIGHT_PWM_DIO_CHANNEL = 8;                // DIO channel used for the absolute encoder (blue)
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;                 // Motor controller CAN ID AND PDP Port number
        public static final int FRONT_RIGHT_TURN_MOTOR_ID = 2;                  // Motor controller CAN ID AND PDP Port number

        public static final int REAR_RIGHT_QUAD_A_DIO_CHANNEL = 0;              // DIO channel used for the quadrature encoder signal A (yellow)
        public static final int REAR_RIGHT_QUAD_B_DIO_CHANNEL = 1;              // DIO channel used for the quadrature encoder signal B (green)
        public static final int REAR_RIGHT_PWM_DIO_CHANNEL = 2;                 // DIO channel used for the absolute encoder (blue)
        public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 20;                 // Motor controller CAN ID AND PDP Port number (port 0=20)
        public static final int REAR_RIGHT_TURN_MOTOR_ID = 1;                   // Motor controller CAN ID AND PDP Port number

        public static final int LEFT_JOYSTICK = 0;                              // Left joystick port number
        public static final int RIGHT_JOYSTICK = 1;                             // Right joystick port number
        public static final int DRIVER_BUTTON_BOARD = 2;                        // Drive button controller port number


    }


    /**
     * Calibrated values and constants of the drivetrain.
     */
    public static final class DRIVETRAIN {
        // These are the design constants
        public static final double DRIVE_GEAR_RATIO = 8.16;                     // The drive gear ration - MK3 Standard = 8.16
        public static final double TURN_GEAR_RATIO = 12.8;                      // The turning gear ration - MK3 = 12.8
        
        // These are the calibrated values...these should be updated before
        // each event.
        public static final double WHEEL_DIAMETER_INCH = 4.0;                   // Average wheel diameter (best: calibrate with encoders, ok: use tape measure)
        public static final double TRACK_WIDTH_INCH = 18.0;                     // Distance between right and left wheels (best: calibrate with encoders, ok: use tape measure)
        public static final double WHEEL_BASE_INCH = 17.125;                    // Distance between front and back wheels (best: calibrate with encoders, ok: use tape measure)
        public static final double TURN_P_GAIN = 0.7;                           // The P-gain of the PID turning controller
        public static final double TURN_D_GAIN = 0.01;                          // The D-gain of the PID turning controller
        public static final double MAX_TURN_ERROR_DEG = 3.0;                    // The maximum turning controller error considered on-target
        public static final double DRIVE_P_GAIN = 0.1;                          // The P-gain of the PID driving controller
        public static final double DRIVE_D_GAIN = 0.0;                          // The D-gain of the PID driving controller
        public static final double MAX_TURN_VELOCITY_RPS = 20*Math.PI;          // The maximum angular velocity - used for constraining trajectory profile
        public static final double MAX_TURN_ACCELERATION_RPSS = 20*Math.PI;     // The maximum angular acceleration - used for constraining trajectory profile
        public static final double MAX_DRIVE_VELOCITY_MPS = 2.5;                // The maximum drive velocity

        // And these should be checked before every match.
        public static final double FRONT_LEFT_ZERO_RAD = 3.11;                  // The PWM encoder angle for zero'ing the wheel
        public static final double FRONT_RIGHT_ZERO_RAD = 3.43;                 // The PWM encoder angle for zero'ing the wheel
        public static final double REAR_LEFT_ZERO_RAD = 2.33;                   // The PWM encoder angle for zero'ing the wheel
        public static final double REAR_RIGHT_ZERO_RAD = 0.14;                  // The PWM encoder angle for zero'ing the wheel
        
    }

    /**
     * These should be updated based on what the driver wants or is capable
     * of using.
     */
    public static final class DRIVER {
        public static final double MAX_DRIVE_VELOCITY = 2.5;                // The maximum drive velocity
        public static final double MAX_ROTATION_VELOCITY = Math.PI;              // The maximum drive rotation velocity
        public static final double JOYSTICK_DEADBAND = 0.02;                    // The joystick deadband
        public static final int DRIVE_SLEW_RATE_LIMITER = 3;                    // The units per second max change
    }


    public static final class CONTROL {
        public static final double LOOP_TIME_S = 0.005;                         // The update manager loop time
    }



}
