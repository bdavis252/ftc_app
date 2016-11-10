package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by djordan on 10/25/16.
 */

@Autonomous(name="AutoRed", group="Pushbot")

public class AutoRedLong1617 extends LinearOpMode {

    /* Declare OpMode members. */
    //HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 3.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 5.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    boolean isRed = true;

    ColorSensor colorSensor;
    DcMotor rightSide;
    DcMotor leftSide;
    Servo beaconArm;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // robot.init(hardwareMap);
        boolean bLedOn = false;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftSide = hardwareMap.dcMotor.get("leftSide");
        rightSide = hardwareMap.dcMotor.get("rightSide");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        leftSide.setDirection(DcMotor.Direction.REVERSE);

        leftSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        // telemetry.addData("Path0",  "Starting at %7d :%7d",
        // robot.leftMotor.getCurrentPosition(),
        // robot.rightMotor.getCurrentPosition());
        // telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        colorSensor.enableLed(bLedOn);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, -101, -101, 9.0);  // S3: Reverse 24 Inches with 6 Sec timeout
        if (isRed == true)
            encoderDrive(TURN_SPEED,   3, -3, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        else
            encoderDrive(TURN_SPEED,   -3, 3, 4.0);

        if (colorSensor.red() > 4) {
            beaconArm.setPosition(1);
        }
        if (colorSensor.blue() > 4) {
            beaconArm.setPosition(0);
        }

        // robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        // robot.rightClaw.setPosition(0.0);
        // sleep(1000);     // pause for servos to move

        // telemetry.addData("Path", "Complete");
        // telemetry.update();

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftSide.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightSide.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftSide.setTargetPosition(newLeftTarget);
            rightSide.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftSide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftSide.setPower(Math.abs(speed));
            rightSide.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftSide.isBusy() && rightSide.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftSide.getCurrentPosition(),
                        rightSide.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftSide.setPower(0);
            rightSide.setPower(0);

            // Turn off RUN_TO_POSITION
            leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}