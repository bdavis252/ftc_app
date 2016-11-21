package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by djordan on 10/25/16.
 */

@Autonomous(name="Auto Red Long", group="Pushbot")

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

    static final double     APPROACH_SPEED  = 0.5;
    static final double     WHITE_THRESHOLD = 0.2;

    static boolean isRed;

    static ColorSensor colorSensor;
    static DcMotor rightSide;
    static DcMotor leftSide;
    Servo beaconArm;
    DcMotor Shooter;
    static boolean bLedOn = false;

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;


    VuforiaLocalizer vuforia;

    static boolean first = true;

    @Override
    public void runOpMode() {

        if ( first == true) {
            init5754(leftSide, rightSide);
            first = false;
        }

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;

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

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, -101, -101, 9.0);  // S3: Reverse 24 Inches with 6 Sec timeout
        if (isRed == true)
            encoderDrive(TURN_SPEED,   3, -3, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        else
            encoderDrive(TURN_SPEED,   -3, 3, 4.0);

        Orientation orientation = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        leftSide.setPower(APPROACH_SPEED);
        rightSide.setPower(APPROACH_SPEED);
        while (opModeIsActive() && (orientation.firstAngle < WHITE_THRESHOLD)) {

            telemetry.addData("Light Level", orientation.secondAngle);
            telemetry.update();
        }


        leftSide.setPower(0);
        rightSide.setPower(0);

        if (colorSensor.red() > 4 && isRed == true) {
            beaconArm.setPosition(1);
        }
        if (colorSensor.blue() > 4 && isRed == false) {
            beaconArm.setPosition(0);
        }

        // robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        // robot.rightClaw.setPosition(0.0);
        // sleep(1000);     // pause for servos to move

        // telemetry.addData("Path", "Complete");
        // telemetry.update();

        // Start Vuforia Code
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATDPvOb/////AAAAGe9bQXYDrkV8oqDe7ws2FjcrmG2MHmLVfTLrodv1HN8P0VshFm2K79Qzd9PeHXqnNLTw7+nrW35I2NLRvq64OC/dy5pgW81Ms++JNKktwX3npJf43CCoKSFEyEgqNfKzePAky1Qz8QTMQgOuxU2zmSdeNpy0Xl5yh0Sep86nFbjm4c5/yI/zpLrRWnRNbfjAZXuzaYi+Id4RUhHDgq919a2fDiZfOE7R56zCp4e4iRmQQcmeRJXsLUzyLbwFKM3rwJoCUijx2rhgwKUAhAdWrxHxQqRMbQWlLdcIbxRB9ZqPWxeybS0h2AXy/IzMIp1aUBa1V9uzET9XyB+vVgO5WKeVsy+4iFtE1QwqQcPA6P47";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables stonesAndChips = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable redTarget = stonesAndChips.get(0);
        redTarget.setName("RedTarget");  // Wheels


        VuforiaTrackable redTarget2 = stonesAndChips.get(1);
        redTarget2.setName("RedTarget2"); // Legos

        VuforiaTrackable blueTarget  = stonesAndChips.get(2);
        blueTarget.setName("BlueTarget");  // Tools

        VuforiaTrackable blueTarget2  = stonesAndChips.get(3);
        blueTarget2.setName("BlueTarget2"); // Gears

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(stonesAndChips);


        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        OpenGLMatrix redTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0))
                .translation(0, mmFTCFieldWidth / 6,0);
        redTarget.setLocation(redTargetLocationOnField);
        RobotLog.ii(TAG, "Red Target=%s", format(redTargetLocationOnField));

        OpenGLMatrix redTargetLocationOnField2 = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0))
                .translation(0, -mmFTCFieldWidth / 6, 0);
        redTarget.setLocation(redTargetLocationOnField2);
        RobotLog.ii(TAG, "Red Target2=%s", format(redTargetLocationOnField2));

       /*
        * To place the Stones Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
        OpenGLMatrix blueTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0))
                .translation(mmFTCFieldWidth / 6, 0, 0);
        blueTarget.setLocation(blueTargetLocationOnField);
        RobotLog.ii(TAG, "Blue Target=%s", format(blueTargetLocationOnField));

        OpenGLMatrix blueTargetLocationOnField2 = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0))
                .translation(-mmFTCFieldWidth / 6,0, 0);
        blueTarget.setLocation(blueTargetLocationOnField2);
        RobotLog.ii(TAG, "Blue Target2l/`=%s", format(blueTargetLocationOnField2));

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)redTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)redTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        stonesAndChips.activate();

        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {

                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }

            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", format(lastLocation));
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
        }

    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
        // End Vuforia Code

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

             sleep(250);   // optional pause after each move
        }


    }

    public void init5754(DcMotor leftSide, DcMotor rightSide) {
    leftSide.setDirection(DcMotor.Direction.REVERSE);

        colorSensor.enableLed(bLedOn);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftSide = hardwareMap.dcMotor.get("leftSide");
        rightSide = hardwareMap.dcMotor.get("rightSide");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        if (gamepad1.x)
            isRed = true;
        else if (gamepad1.y)
            isRed = false;


}
}


