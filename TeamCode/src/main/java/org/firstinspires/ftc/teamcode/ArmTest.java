package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp (name = "KRC2025", group = "2025KRC OPMODE")


public class ArmTest extends LinearOpMode {

    private DcMotorEx AL;
    private DcMotorEx AA;

    @Override

    public void runOpMode () throws InterruptedException {

        AL = hardwareMap.get(DcMotorEx.class,"AL"); //arm lengh
        AA = hardwareMap.get(DcMotorEx.class,"AA"); //arm angle

        AL.setDirection(DcMotorSimple.Direction.REVERSE);
        AA.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        imu.initialize(parameters);

        AL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        int targetLengh = 0;
        int targetAngle = 0;
        int currentLengh = 0;
        int currnetAngle = 0;

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (gamepad1.y) {
                targetLengh = 400;
                AL.setTargetPosition(targetLengh);
                AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AL.setPower(0.5);
                currentLengh = AL.getCurrentPosition();
            }

            if (gamepad1.x) {
                targetLengh = currentLengh + 100;
                AL.setTargetPosition(targetLengh);
                AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AL.setPower(0.5);
                currentLengh = AL.getCurrentPosition();
            }

            if (gamepad1.b) {
                if (targetLengh > 10) {
                    targetLengh = currentLengh - 100;
                    AL.setTargetPosition(targetLengh);
                    AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AL.setPower(0.5);
                    currentLengh = AL.getCurrentPosition();
                } else {
                    targetLengh = 10;
                    AL.setTargetPosition(targetLengh);
                    AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AL.setPower(0.5);
                    currentLengh = AL.getCurrentPosition();
                }
            }

            if (gamepad1.a) {
                targetLengh = 0;
                AL.setTargetPosition(targetLengh);
                AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AL.setPower(0.5);
                currentLengh = AL.getCurrentPosition();
            }

            if (gamepad1.dpad_up) {
                targetAngle = 400;
                AA.setTargetPosition(targetAngle);
                AA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AA.setPower(0.5);
                currnetAngle = AA.getCurrentPosition();
            }

            if (gamepad1.dpad_left) {
                targetAngle = currnetAngle + 100;
                AA.setTargetPosition(targetAngle);
                AA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AA.setPower(0.5);
                currnetAngle = AA.getCurrentPosition();
            }

            if (gamepad1.dpad_right) {
                if (targetAngle > 10) {
                    targetAngle = currnetAngle - 100;
                    AA.setTargetPosition(targetAngle);
                    AA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AA.setPower(0.5);
                    currnetAngle = AA.getCurrentPosition();
                } else {
                    targetAngle = 10;
                    AA.setTargetPosition(targetAngle);
                    AA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AA.setPower(0.5);
                    currnetAngle = AA.getCurrentPosition();
                }
            }

            if (gamepad1.dpad_down) {
                targetAngle = 0;
                AA.setTargetPosition(targetAngle);
                AA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AA.setPower(0.5);
                currnetAngle = AA.getCurrentPosition();
            }



            telemetry.addData("AL", AL.getCurrentPosition());
            telemetry.addData("AA", AA.getCurrentPosition());
            telemetry.update();
            }



        }

    }