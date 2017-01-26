package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by cjames on 10/25/16.
 */
@TeleOp(name = "CDrive", group = "Iterative OpMode")

public class CameronDriveTryTwo extends OpMode {
    DcMotor rightSide;
    DcMotor leftSide;
    DcMotor beaconArm;
    DcMotor Sweeper;
    DcMotor Shooter1;
    DcMotor Shooter2;

    //double leftPower = gamepad1.left_stick_y;
    //double rightPower = gamepad1.right_stick_y;

    public CameronDriveTryTwo() {
    }

    @Override
    public void init() {
        rightSide = hardwareMap.dcMotor.get("motor1");
        leftSide = hardwareMap.dcMotor.get("motor2");
        leftSide.setDirection(DcMotor.Direction.REVERSE);
        beaconArm = hardwareMap.dcMotor.get("beacon");
        Sweeper = hardwareMap.dcMotor.get("ballSweeper");
        Sweeper.setDirection(DcMotor.Direction.REVERSE);
        Shooter1 = hardwareMap.dcMotor.get("shooter1");
        Shooter2 = hardwareMap.dcMotor.get("shooter2");
        Shooter2.setDirection(DcMotor.Direction.REVERSE);

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


        if (gamepad1.a) {
            beaconArm.setPower(1);
        }
        else if (gamepad1.b) {
            beaconArm.setPower(-1);
        }




        if (gamepad2.right_bumper) {
            Sweeper.setPower(1);
        }
        else if (gamepad2.y)
            Sweeper.setPower(-1);
        else
            Sweeper.setPower(0);

        if (gamepad2.left_bumper) {
            Shooter1.setPower(1);
            Shooter2.setPower(1);
        }
        else
            Shooter1.setPower(0);
            Shooter2.setPower(0);

    }

        @Override
        public void stop () {

        }
    }

