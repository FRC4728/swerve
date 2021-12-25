package frc.robot;

public final class Constants {

    public static final class HARDWARE {
        public static final int FRONT_LEFT_QUAD_A_DIO_CHANNEL = 9;              // DIO channel used for the quadrature encoder signal A (yellow)
        public static final int FRONT_LEFT_QUAD_B_DIO_CHANNEL = 10;             // DIO channel used for the quadrature encoder signal B (green) (on MXP)
        public static final int FRONT_LEFT_PWM_DIO_CHANNEL = 11;                // DIO channel used for the absolute encoder (blue) (on MXP)
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 12;                 // Motor controller CAN ID AND PDP Port number
        public static final int FRONT_LEFT_TURN_MOTOR_ID = 13;                  // Motor controller CAN ID AND PDP Port number

        public static final int REAR_LEFT_QUAD_A_DIO_CHANNEL = 3;               // DIO channel used for the quadrature encoder signal A
        public static final int REAR_LEFT_QUAD_B_DIO_CHANNEL = 4;               // DIO channel used for the quadrature encoder signal B
        public static final int REAR_LEFT_PWM_DIO_CHANNEL = 5;                  // DIO channel used for the absolute encoder
        public static final int REAR_LEFT_DRIVE_MOTOR_ID = 15;                  // Motor controller CAN ID AND PDP Port number
        public static final int REAR_LEFT_TURN_MOTOR_ID = 14;                   // Motor controller CAN ID AND PDP Port number

        public static final int FRONT_RIGHT_QUAD_A_DIO_CHANNEL = 6;             // DIO channel used for the quadrature encoder signal A
        public static final int FRONT_RIGHT_QUAD_B_DIO_CHANNEL = 7;             // DIO channel used for the quadrature encoder signal B
        public static final int FRONT_RIGHT_PWM_DIO_CHANNEL = 8;                // DIO channel used for the absolute encoder
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;                 // Motor controller CAN ID AND PDP Port number
        public static final int FRONT_RIGHT_TURN_MOTOR_ID = 2;                  // Motor controller CAN ID AND PDP Port number

        public static final int REAR_RIGHT_QUAD_A_DIO_CHANNEL = 0;              // DIO channel used for the quadrature encoder signal A
        public static final int REAR_RIGHT_QUAD_B_DIO_CHANNEL = 1;              // DIO channel used for the quadrature encoder signal B
        public static final int REAR_RIGHT_PWM_DIO_CHANNEL = 2;                 // DIO channel used for the absolute encoder
        public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 20;                 // Motor controller CAN ID AND PDP Port number (port 0=20)
        public static final int REAR_RIGHT_TURN_MOTOR_ID = 1;                   // Motor controller CAN ID AND PDP Port number

    }

    public static final class DRIVETRAIN {
        public static final double GEAR_RATIO = 8.16;                // MK3 Fast = 6.86, MK3 Standard = 8.16
        public static final double WHEEL_DIAMETER_INCH = 4.0;                   // Average wheel diameter (should all be very, very close)
        public static final double TRACK_WIDTH_INCH = 18.0;                         // Distance between centers of right and left wheels on robot in meters
        public static final double WHEEL_BASE_INCH = 17.125;                          // Distance between front and back wheels on robot in meters
        public static final double FRONT_LEFT_ZERO_RAD = 3.201;                 // The PWM encoder angle for zero'ing the wheel
        public static final double REAR_LEFT_ZERO_RAD = 2.562;                 // The PWM encoder angle for zero'ing the wheel
        public static final double FRONT_RIGHT_ZERO_RAD = 3.455;               // The PWM encoder angle for zero'ing the wheel
        public static final double REAR_RIGHT_ZERO_RAD = 0.223;                 // The PWM encoder angle for zero'ing the wheel
        public static final double TURN_P_GAIN = 0.0;                // The P-gain of the PID turning controller
        public static final double TURN_D_GAIN = 0.0;                // The D-gain of the PID turning controller
        public static final double DRIVE_P_GAIN = 0.0;               // The P-gain of the PID driving controller
        public static final double DRIVE_D_GAIN = 0.0;               // The D-gain of the PID driving controller
        public static final double MAX_TURN_VELOCITY_DPS = 0.0;        // The maximum angular velocity in radians per second
        public static final double MAX_TURN_ACCELERATION_DPSS = 0.0;        // The maximum angular acceleration in radians per second
        public static final double MAX_DRIVE_VELOCITY_FPS = 0.0;          // The maximum drive velocity in meters per second
        public static final double MAX_DRIVE_ACCELERATION_FPSS = 0.0;          // The maximum drive velocity in meters per second

    }

    public static final class DRIVER {
        public static final int JOYSTICK_TURN = 0;                              // Turn joystick port number
        public static final int JOYSTICK_THROTTLE = 1;                          // Throttle joystick port number
        public static final int DRIVER_BUTTON_BOARD = 2;                        // Drive button controller port number
    }


}
