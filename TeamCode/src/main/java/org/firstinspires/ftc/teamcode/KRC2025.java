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


public class KRC2025 extends LinearOpMode {

    private DcMotorEx AL;
    private DcMotorEx AA;

    private Servo gripper;
    private Servo wristL;
    private Servo wristR;


    @Override

    public void runOpMode () throws InterruptedException {
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        AL = hardwareMap.get(DcMotorEx.class,"AL"); //arm lengh
        AA = hardwareMap.get(DcMotorEx.class,"AA"); //arm angle

        AL.setDirection(DcMotorSimple.Direction.REVERSE);
        AA.setDirection(DcMotorSimple.Direction.REVERSE);


        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        gripper = hardwareMap.servo.get("gripper");
        wristL = hardwareMap.servo.get("wristL");
        wristR = hardwareMap.servo.get("wristR");

        wristL.setDirection(Servo.Direction.REVERSE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        imu.initialize(parameters);

        waitForStart();

        int targetLengh = 0;
        int targetAngle = 0;
        int currentLengh = 0;
        int currentAngle = 0;

        gripper.setPosition(0.1);
        wristL.setPosition(0.22);
        wristR.setPosition(0.22);

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            int ArmLengh = AL.getCurrentPosition();
            int ArmAngle = AA.getCurrentPosition();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double Y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double X = gamepad1.left_stick_x;
            double R = gamepad1.right_stick_x;
            double slow = 1 - (0.7 * gamepad1.right_trigger);

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = X * Math.cos(-botHeading) - Y * Math.sin(-botHeading);
            double rotY = X * Math.sin(-botHeading) + Y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(R), 1);
            double leftFrontPower = ((rotY + rotX + R) / denominator) * slow;
            double leftBackPower = ((rotY - rotX + R) / denominator) * slow;
            double rightFrontPower = ((rotY - rotX - R) / denominator) * slow;
            double rightBackPower  = ((rotY + rotX - R) / denominator) * slow;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            //gripper open/close
            if (gamepad1.right_bumper) {
                gripper.setPosition(0.3);
            } else {
                gripper.setPosition(0.1);
            }

            //구조물 안의 기물 잡을 준비
            if (gamepad1.y) {
                targetAngle = 100;
                AA.setTargetPosition(targetAngle);
                AA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AA.setPower(0.5);
                currentAngle = AA.getCurrentPosition();
                targetLengh = 200;
                AL.setTargetPosition(targetLengh);
                AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AL.setPower(0.5);
                currentLengh = AL.getCurrentPosition();
                wristL.setPosition(0.5);
                wristR.setPosition(0.5);
            }

            //구조물 안의 기물 잡기 / 손목 수평
            if (gamepad1.x) {
                wristL.setPosition(0.75);
                wristR.setPosition(0.75);
            }

            //준비
            if (gamepad1.a) {
                wristL.setPosition(0.22);
                wristR.setPosition(0.22);
                targetLengh = 0;
                AL.setTargetPosition(targetLengh);
                AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AL.setPower(0.5);
                currentLengh = AL.getCurrentPosition();
                targetAngle = 0;
                AA.setTargetPosition(targetAngle);
                AA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AA.setPower(0.5);
                currentAngle = AA.getCurrentPosition();
            }

            //구조물 안의 기물 잡기 / 손목 수직
            if (gamepad1.b) {
                wristL.setPosition(0.9);
                wristR.setPosition(0.6);
            }

            //2층 체임버 높이
            if (gamepad1.dpad_left) {
                targetLengh = 100;
                AL.setTargetPosition(targetLengh);
                AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AL.setPower(0.5);
                currentLengh = AL.getCurrentPosition();
                targetAngle = 50;
                AA.setTargetPosition(targetAngle);
                AA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AA.setPower(0.5);
                currentAngle = AA.getCurrentPosition();
            }

            //1층 체임버 높이
            if (gamepad1.dpad_right) {
            }

            //2층 바구니 높이
            if (gamepad1.dpad_up) {
                targetAngle = 400;
                AA.setTargetPosition(targetLengh);
                AA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AA.setPower(0.5);

            }

            //1층 바구니 높이
            if (gamepad1.dpad_down) {
            }

            //팔 길이 확장
            if (gamepad2.x) {
                targetLengh = currentLengh + 50;
                AL.setTargetPosition(targetLengh);
                AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AL.setPower(0.5);
                currentLengh = AL.getCurrentPosition();
            }

            //팔 길이 축소
            if (gamepad2.b) {
                if (targetLengh > 10) {
                    targetLengh = currentLengh - 50;
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

            //팔 각도 올리기
            if (gamepad2.y) {
                targetAngle = currentAngle + 50;
                AA.setTargetPosition(targetAngle);
                AA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AA.setPower(0.5);
                currentAngle = AA.getCurrentPosition();
            }

            //팔 각도 내리기
            if (gamepad2.a) {
                if (targetAngle > 10) {
                    targetAngle = currentAngle - 50;
                    AA.setTargetPosition(targetAngle);
                    AA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AA.setPower(0.5);
                    currentAngle = AA.getCurrentPosition();
                } else {
                    targetAngle = 10;
                    AA.setTargetPosition(targetAngle);
                    AA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AA.setPower(0.5);
                    currentAngle = AA.getCurrentPosition();
                }
            }

            telemetry.addData("gripper", gripper.getPosition());
            telemetry.addData("wristL", wristL.getPosition());
            telemetry.addData("wristR",wristR.getPosition());
            telemetry.addData("ArmLengh", currentLengh);
            telemetry.addData("ArmAngle", currentAngle);
            telemetry.update();

        }

    }
}