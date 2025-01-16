package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
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

    private com.arcrobotics.ftclib.controller.PIDController controller_AA;

    public static double p_AA = 0, i_AA = 0, d_AA = 0;
    public static double f_AA = 0;

    public static int target_AA = 0;

    private final double ticks_in_degree_AA = 800 / 180.0;


    private com.arcrobotics.ftclib.controller.PIDController controller_AL;

    public static double p_AL = 0, i_AL = 0, d_AL = 0;
    public static double f_AL = 0;

    public static int target_AL = 0;

    private final double ticks_in_degree_AL = 700 / 180.0;


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

        double WlPosion = 0;
        double WrPosion = 0;
        double interval = 0.05;

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

            if (gamepad1.left_bumper) {
                wristL.setPosition(0.5);
                wristR.setPosition(0.5);
            }

            //준비
            if (gamepad1.y) {
                wristL.setPosition(0.22);
                wristR.setPosition(0.22);
            }

            //손목 반시계 방향으로 돌리기
            if (rising_edge(currentGamepad1.x, previousGamepad1.x)) {
                WlPosion = wristL.getPosition() + interval;
                WrPosion = wristR.getPosition() - interval;

                if (WlPosion > 1) {
                    WlPosion = 1;
                }

                if (WrPosion < 0) {
                    WrPosion = 0;
                }
                wrist_control(WlPosion, WrPosion);
            }

            //손목 시계 방향으로 돌리기
            if (rising_edge(currentGamepad1.b, previousGamepad1.b)){
                WlPosion = wristL.getPosition() - interval;
                WrPosion = wristR.getPosition() + interval;

                if (WlPosion < 0) {
                    WlPosion = 0;
                }

                if (WrPosion > 1) {
                    WrPosion = 1;
                }
                wrist_control(WlPosion, WrPosion);
            }


            //구조물 안의 기물 잡기
            if (gamepad1.a) {
                wristL.setPosition(0.75);
                wristR.setPosition(0.75);
            }


            //팔 각도 올리기
            if (gamepad1.dpad_left) {
                targetAngle = currentAngle + 100;
                AA.setTargetPosition(targetAngle);
                AA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AA.setPower(0.5);
                currentAngle = AA.getCurrentPosition();
            }

            //팔 각도 내리기
            if (gamepad1.dpad_right) {
                if (targetAngle > 10) {
                    targetAngle = currentAngle - 100;
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

            //팔 길이 늘리기
            if (gamepad1.dpad_up) {
                targetLengh = currentLengh + 100;
                AL.setTargetPosition(targetLengh);
                AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AL.setPower(0.5);
                currentLengh = AL.getCurrentPosition();
            }

            //팔 길이 줄이기
            if (gamepad1.dpad_down) {
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

            telemetry.addData("gripper", gripper.getPosition());
            telemetry.addData("wristL", wristL.getPosition());
            telemetry.addData("wristR",wristR.getPosition());
            telemetry.addData("ArmLengh", currentLengh);
            telemetry.addData("ArmAngle", currentAngle);
            telemetry.update();

        }
    }
    private void wrist_control(double L, double R) {
        wristL.setPosition(L);
        wristR.setPosition(R);
    }

    private boolean rising_edge(boolean currentButtonState, boolean previousButtonState) {
        return currentButtonState && !previousButtonState;
    }

}