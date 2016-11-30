/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

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


@Autonomous(name="Vuforia Navigation", group ="Concept")
public class VuforiaNavigation1617 extends LinearOpMode {

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;


    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

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
        blueTarget.setName("BlueTarget");  // Tools //TODO: Actually this is Legos and the one above is Tools.

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


        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix //TODO: don't forget to place the phone properly once it's time.
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

                //TODO: TEAM, THIS IS THIS PROPER WAY TO GET THE ANGLES AS FLOATS (NUMBERS)
                Orientation orientation = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                telemetry.addData("angle1test", orientation.firstAngle);
                telemetry.addData("angle2test", orientation.secondAngle);
                telemetry.addData("angle3test", orientation.thirdAngle);
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
