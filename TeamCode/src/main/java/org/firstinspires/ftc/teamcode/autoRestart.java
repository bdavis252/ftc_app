package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.*;
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
 *
 */
@Autonomous(name="Auto try 2", group="Pushbot")
public class autoRestart extends LinearOpMode {
    /* Declare OpMode members. */
    //HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 5.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    DcMotor rightSide;
    DcMotor leftSide;
    Orientation orientation;
    VuforiaTrackables stonesAndChips;


    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    public static int first = 0;

    @Override
    public void runOpMode() {

        if (!opModeIsActive()){
            leftSide = hardwareMap.dcMotor.get("motor2");
            rightSide = hardwareMap.dcMotor.get("motor1");
            leftSide.setDirection(DcMotor.Direction.REVERSE);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");
            //telemetry.update();


            leftSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "ATDPvOb/////AAAAGe9bQXYDrkV8oqDe7ws2FjcrmG2MHmLVfTLrodv1HN8P0VshFm2K79Qzd9PeHXqnNLTw7+nrW35I2NLRvq64OC/dy5pgW81Ms++JNKktwX3npJf43CCoKSFEyEgqNfKzePAky1Qz8QTMQgOuxU2zmSdeNpy0Xl5yh0Sep86nFbjm4c5/yI/zpLrRWnRNbfjAZXuzaYi+Id4RUhHDgq919a2fDiZfOE7R56zCp4e4iRmQQcmeRJXsLUzyLbwFKM3rwJoCUijx2rhgwKUAhAdWrxHxQqRMbQWlLdcIbxRB9ZqPWxeybS0h2AXy/IzMIp1aUBa1V9uzET9XyB+vVgO5WKeVsy+4iFtE1QwqQcPA6P47";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);


            VuforiaTrackables stonesAndChips = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
            VuforiaTrackable redTarget = stonesAndChips.get(3);
            redTarget.setName("RedTarget");  // Gears


            VuforiaTrackable redTarget2 = stonesAndChips.get(1);
            redTarget2.setName("RedTarget2"); // Tools

            VuforiaTrackable blueTarget = stonesAndChips.get(2);
            blueTarget.setName("BlueTarget");  // Legos

            VuforiaTrackable blueTarget2 = stonesAndChips.get(0);
            blueTarget2.setName("BlueTarget2"); // Wheels


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
                            AngleUnit.DEGREES, 90, 90, 0));
            redTarget.setLocation(redTargetLocationOnField);
            RobotLog.ii(TAG, "Red Target=%s", format(redTargetLocationOnField));

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
                            AngleUnit.DEGREES, 90, 0, 0));
            blueTarget.setLocation(blueTargetLocationOnField);
            RobotLog.ii(TAG, "Blue Target=%s", format(blueTargetLocationOnField));


            OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                    .translation(mmBotWidth/2,0,0)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.YZY,
                            AngleUnit.DEGREES, -90, 0, 0));
            RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));


            ((VuforiaTrackableDefaultListener)redTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener)blueTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);


            telemetry.addData(">", "Press Play to start tracking");
            telemetry.update();

        }




        // Send telemetry message to indicate successful Encoder reset
        // telemetry.addData("Path0",  "Starting at %7d :%7d",
        // robot.leftMotor.getCurrentPosition(),
        // robot.rightMotor.getCurrentPosition());
        // telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

       // move
        encoderDrive(DRIVE_SPEED, 36, -36, 9.0);  // drive 36 in in 9 sec
        encoderDrive(TURN_SPEED,   0, -0, 0);
        stonesAndChips.activate();


        boolean doLoop = true;
        while (opModeIsActive() && (doLoop)){


              for (VuforiaTrackable trackable : allTrackables) {

                  telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                  OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                  if (robotLocationTransform != null) {
                      lastLocation = robotLocationTransform;
                  }
              }
              if (lastLocation != null) {
                  //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                  telemetry.addData("Pos", format(lastLocation));

                  //TODO: TEAM, THIS IS THIS PROPER WAY TO GET THE ANGLES AS FLOATS (NUMBERS)

                  orientation = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                  telemetry.addData("angle1test", orientation.firstAngle);
                  telemetry.addData("angle2test", orientation.secondAngle);
                  telemetry.addData("angle3test", orientation.thirdAngle);

                  if (orientation.secondAngle > -90)
                      doLoop = false;
              }
              else {
                  telemetry.addData("Pos", "Unknown");
              }
              telemetry.update();

          }


        }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();

    }


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




    }


