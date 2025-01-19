package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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


@TeleOp (name = "2025KRC", group = "2025KRC OPMODE")


public class KRC2025 extends LinearOpMode {

    private PIDController controller;

    public static double p = 0.01, i = 0, d = 0.0005; //초기값 0.005
    public static double f = 0.2;//초기값 0.28
    public static double maxSpeed = 0.9; // 최대 속도 제한
    public static double maxAccel = 0.1; // 최대 가속도 제한
    public static double sCurveRampRate = 0.04; // S-curve 변화율 (0 ~ 1)
    private double currentTarget = 0; // 점진적으로 변경되는 목표값
    private double previousPower = 0; // 이전 루프의 출력값 저장


    public static int target = 0;

    private final int targetUp = 420; // D패드 업 시 목표값
    private final int targetDown = 0; // D패드 다운 시 목표값
    private final int targetC = 600; // 체임버에 기물 거는 높이
    private final int targetG = 100; // 휴먼플레이어가 주는 기물 잡는 높이
    private final int targetGUP = 300; // 걸린 기물 잡고 팔 올리는 값
    private final int LenghUP = 4000; // 2층 바구니로 팔 올리는 값
    private final int LenghDown = 0; // 팔 길이 최소값

    private final int targetSUP = 50; //팔 각도 약간 올리는 값

    private final double ticks_in_degree_AA = 800 / 180.0;

    private DcMotorEx AL;
    private DcMotorEx AF;
    private DcMotorEx AB;

    private Servo gripper;
    private Servo wristL;
    private Servo wristR;

    //그리퍼 오픈 클로즈 상태값
    private double gripOepn = 0.31;
    private double gripClose = 0.08;


    @Override

    public void runOpMode () throws InterruptedException {

        controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AL = hardwareMap.get(DcMotorEx.class, "AL");
        AF = hardwareMap.get(DcMotorEx.class, "AF");
        AB = hardwareMap.get(DcMotorEx.class, "AB");

        AL.setDirection(DcMotorSimple.Direction.REVERSE);


        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

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
        int currentLengh = 0;

        double WlPosion = 0;
        double WrPosion = 0;
        double interval = 0.05;

        gripper.setPosition(gripClose);
        wristL.setPosition(0.22);
        wristR.setPosition(0.22);

        AL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        while (opModeIsActive()) {

            if (isStopRequested()) return;

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
                gripper.setPosition(gripOepn);
            } else {
                gripper.setPosition(gripClose);
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

            if (gamepad1.dpad_left) {
                targetLengh = currentLengh + 100;
                AL.setTargetPosition(targetLengh);
                AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AL.setPower(1);
                currentLengh = AL.getCurrentPosition();
            }

            if (gamepad1.dpad_right) {
                if (targetLengh>10) {
                    targetLengh =currentLengh - 100;
                    AL.setTargetPosition(targetLengh);
                    AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AL.setPower(1);
                    currentLengh = AL.getCurrentPosition();
                } else {
                    targetLengh = 10;
                    AL.setTargetPosition(targetLengh);
                    AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AL.setPower(1);
                    currentLengh = AL.getCurrentPosition();
                }
            }

            if (gamepad2.dpad_up) {
                targetLengh = LenghUP;
                AL.setTargetPosition(targetLengh);
                AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AL.setPower(1);
                currentLengh = AL.getCurrentPosition();
            }

            if (gamepad2.dpad_down) {
                targetLengh = LenghDown;
                AL.setTargetPosition(targetLengh);
                AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                AL.setPower(1);
                currentLengh = AL.getCurrentPosition();
            }

            if (gamepad2.y) {
                target = targetUp;
            } else if (gamepad2.a) {
                target = targetDown;
            }

            if (gamepad2.x) {
                target = targetG;
            } else if (gamepad2.b) {
                target = targetC;
                wristL.setPosition(0.25);
                wristR.setPosition(0.75);
            }

            if (gamepad2.dpad_left) {
                target = targetGUP;
            }

            if (gamepad2.dpad_right) {
                target = targetSUP;
            }




            double error = target - currentTarget;

            double increment = sCurveRampRate * error;
            if (Math.abs(increment)>Math.abs(error)) {
                increment = error;
            }
            currentTarget += increment;

            int armPos = AF.getCurrentPosition();
            double pid = controller.calculate(armPos,currentTarget);
            double ff = Math.cos(Math.toRadians(armPos/ticks_in_degree_AA)) * f;

            double rawPower = pid + ff;
            rawPower = Math.max(-maxSpeed, Math.min(maxSpeed,rawPower));

            double powerChange = rawPower - previousPower;
            if (Math.abs(powerChange)>maxAccel) {
                powerChange = Math.signum(powerChange) * maxAccel;
            }

            double finalPower = previousPower + powerChange;
            previousPower = finalPower;

            AF.setPower(finalPower);
            AB.setPower(finalPower);

            telemetry.addData("gripper", gripper.getPosition());
            telemetry.addData("wristL", wristL.getPosition());
            telemetry.addData("wristR",wristR.getPosition());
            telemetry.addData("pos_AL ", AL.getCurrentPosition());
            telemetry.addData("Current Target", currentTarget);
            telemetry.addData("Position", armPos);
            telemetry.addData("Target", target);
            telemetry.addData("Power", finalPower);
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