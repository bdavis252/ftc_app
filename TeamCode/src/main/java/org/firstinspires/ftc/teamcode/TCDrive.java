package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by cjames on 10/25/16.
 */
@TeleOp(name = "TCDrive", group = "Iterative OpMode")

public class TCDrive extends OpMode {
    DcMotor rightSide;
    DcMotor leftSide;
    DcMotor beaconArm;
    DcMotor Sweeper;
    DcMotor Shooter1;
    DcMotor Shooter2;
    //double leftPower = gamepad1.left_stick_y;
    //double rightPower = gamepad1.right_stick_y;

    public TCDrive() {
    }

    @Override
    public void init() {
        rightSide = hardwareMap.dcMotor.get("motor1");
        leftSide = hardwareMap.dcMotor.get("motor2");
        leftSide.setDirection(DcMotor.Direction.REVERSE);
        beaconArm = hardwareMap.dcMotor.get("beacon");
        Sweeper = hardwareMap.dcMotor.get("ballSweeper");
        Shooter1 = hardwareMap.dcMotor.get("shooter1");
        Shooter2 = hardwareMap.dcMotor.get("shooter2");

        gamepad1.reset();
        gamepad2.reset();

        rightSide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void loop() {
        // double powerLevel = (Math.sqrt((Math.pow(gamepad1.left_stick_y,2) + Math.pow(gamepad1.right_stick_x,2)))) * Math.sqrt(2);
         rightSide.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) / 2);
         leftSide.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) / 2);


        if (gamepad2.right_bumper) { //TODO needs a stop in code unless we build a mechanical stop.
            beaconArm.setPower(-0.5);
        }
       if (gamepad2.left_bumper) {
            beaconArm.setPower(0.5);
        }
        if (gamepad2.x){
            beaconArm.setPower(0);
        }


        if (gamepad2.a) {
            Sweeper.setPower(-1);
        }
        else
            Sweeper.setPower(0);

        if (gamepad2.b) {
            Shooter1.setPower(1);
            Shooter2.setPower(-1);
        }
        else
            Shooter1.setPower(0);
            Shooter2.setPower(0);

    }

        @Override
        public void stop () {

        }
    }

