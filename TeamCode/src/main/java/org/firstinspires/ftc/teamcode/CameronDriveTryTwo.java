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
    Servo beaconArm;
    DcMotor Sweeper;
    DcMotor Shooter;

    //double leftPower = gamepad1.left_stick_y;
    //double rightPower = gamepad1.right_stick_y;

    public CameronDriveTryTwo() {
    }

    @Override
    public void init() {
        rightSide = hardwareMap.dcMotor.get("motor1");
        leftSide = hardwareMap.dcMotor.get("motor2");
        leftSide.setDirection(DcMotor.Direction.REVERSE);
        beaconArm = hardwareMap.servo.get("servo1");
        Sweeper = hardwareMap.dcMotor.get("ballSweeper");
        Sweeper.setDirection(DcMotor.Direction.REVERSE);
        Shooter = hardwareMap.dcMotor.get("shooter");

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
            beaconArm.setPosition(1);
        }
        if (gamepad1.b) {
            beaconArm.setPosition(0);
        }
        if (gamepad1.x){
            beaconArm.setPosition(.5);
        }


        if (gamepad1.right_bumper) {
            Sweeper.setPower(1);
        }
        else if (gamepad1.y)
            Sweeper.setPower(-1);
        else
            Sweeper.setPower(0);

        if (gamepad1.left_bumper) {
            Shooter.setPower(1);
        }
        else
            Shooter.setPower(0);

    }

        @Override
        public void stop () {

        }
    }

