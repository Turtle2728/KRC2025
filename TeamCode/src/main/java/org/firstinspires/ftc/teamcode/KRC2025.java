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

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        imu.initialize(parameters);

        waitForStart();

        int targetLengh = 0;
        int targetAngle = 0;

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
            }

            //구조물 안의 기물 잡기
            if (gamepad1.a) {
            }

            //준비
            if (gamepad1.x) {
            }

            //걸려있는 기물 잡기
            if (gamepad1.b) {
            }

            //2층 체임버 높이
            if (gamepad1.dpad_left) {
                targetLengh = 450;
                AL.setTargetPosition(targetLengh);
            }

            //1층 체임버 높이
            if (gamepad1.dpad_right) {
            }

            //2층 바구니 높이
            if (gamepad1.dpad_up) {
            }

            //1층 바구니 높이
            if (gamepad1.dpad_down) {
            }

            //팔 길이 확장
            if (gamepad2.x) {
            }

            //팔 길이 축소
            if (gamepad2.y) {
            }

            //손목
            if (gamepad2.a) {
            }

            if (gamepad2.b) {
            }

            telemetry.addData("gripper", gripper.getPosition());
            telemetry.addData("wristL", wristL.getPosition());
            telemetry.addData("wristR",wristR.getPosition());
            telemetry.addData("ArmLengh", ArmLengh);
            telemetry.addData("ArmAngle", ArmAngle);
            telemetry.update();

        }

    }
}