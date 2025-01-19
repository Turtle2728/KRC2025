package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "KRC2025TeleOp", group = "2025KRC OPMODE")
public class KRC2025TeleOp extends OpMode {

    private PIDController controller;

    public static double p = 0.01, i = 0, d = 0.0005;
    public static double f = 0.2;
    public static double maxSpeed = 0.9;
    public static double maxAccel = 0.1;
    public static double sCurveRampRate = 0.04;
    private double currentTarget = 0;
    private double previousPower = 0;

    public static int target = 0;

    private final int targetUp = 420;
    private final int targetDown = 0;
    private final int targetC = 600;
    private final int targetG = 100;

    private final double ticks_in_degree_AA = 800 / 180.0;

    private DcMotorEx AL;
    private DcMotorEx AF;
    private DcMotorEx AB;

    private Servo gripper;
    private Servo wristL;
    private Servo wristR;

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private IMU imu;

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();

    private double WlPosion = 0;
    private double WrPosion = 0;
    private double interval = 0.05;

    //그리퍼 오픈 클로즈 상태값
    private double gripOepn = 0.31;
    private double gripClose = 0.08;

    @Override
    public void init() {

        controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AL = hardwareMap.get(DcMotorEx.class, "AL");
        AF = hardwareMap.get(DcMotorEx.class, "AF");
        AB = hardwareMap.get(DcMotorEx.class, "AB");

        AL.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        gripper = hardwareMap.servo.get("gripper");
        wristL = hardwareMap.servo.get("wristL");
        wristR = hardwareMap.servo.get("wristR");

        wristL.setDirection(Servo.Direction.REVERSE);

        gripper.setPosition(gripClose);
        wristL.setPosition(0.22);
        wristR.setPosition(0.22);

        AL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // gamepad2 Y 버튼을 눌렀을 때 target 설정
        if (gamepad2.y) {
            target = targetUp; // 420
            telemetry.addLine("Y button pressed, target set to targetUp");
        }

        // PID 컨트롤러 및 S-커브 계산
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

        // 모터에 출력 적용
        AF.setPower(finalPower);
        AB.setPower(finalPower);

        // 텔레메트리 출력
        telemetry.addData("Target", target);
        telemetry.addData("Current Target", currentTarget);
        telemetry.addData("Position (AF Encoder)", armPos);
        telemetry.addData("PID Output", pid);
        telemetry.addData("Raw Power", rawPower);
        telemetry.addData("Final Power", finalPower);
        telemetry.addData("AF Power", AF.getPower());
        telemetry.addData("AB Power", AB.getPower());
        telemetry.update();
    }

    private void handleWristRotation() {
        if (risingEdge(currentGamepad1.x, previousGamepad1.x)) {
            WlPosion = wristL.getPosition() + interval;
            WrPosion = wristR.getPosition() - interval;
            WlPosion = Math.min(WlPosion, 1);
            WrPosion = Math.max(WrPosion, 0);
            wrist_control(WlPosion, WrPosion);
        }

        if (risingEdge(currentGamepad1.b, previousGamepad1.b)) {
            WlPosion = wristL.getPosition() - interval;
            WrPosion = wristR.getPosition() + interval;
            WlPosion = Math.max(WlPosion, 0);
            WrPosion = Math.min(WrPosion, 1);
            wrist_control(WlPosion, WrPosion);
        }
    }

    private void wrist_control(double L, double R) {
        wristL.setPosition(L);
        wristR.setPosition(R);
    }

    private boolean risingEdge(boolean current, boolean previous) {
        return current && !previous;
    }
}
