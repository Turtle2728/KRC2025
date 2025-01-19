package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "2025KRC_AUTO", group = "2025KRC_AUTO")
public class KRC2025_AUTO extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.01, i = 0, d = 0.0005; //초기값 0.005
    public static double f = 0.28;//초기값 0.28
    public static double maxSpeed = 0.9; // 최대 속도 제한
    public static double maxAccel = 0.1; // 최대 가속도 제한
    public static double sCurveRampRate = 0.04; // S-curve 변화율 (0 ~ 1)
    private double currentTarget = 0; // 점진적으로 변경되는 목표값
    private double previousPower = 0; // 이전 루프의 출력값 저장

    public static int target = 0;

    private final int targetCUP = 200; // 체임버에 기물 거는 높이

    private final double wristUP = 0.28;

    private final double ticks_in_degree_AA = 800 / 180.0;

    private DcMotorEx AL;
    private DcMotorEx AF;
    private DcMotorEx AB;

    private Servo gripper;
    private Servo wristL;
    private Servo wristR;

    int start_delay = 0;

    public void ALadjust(double armPower, int armTarget) {

        AL.setTargetPosition(armTarget);
        AL.setPower(armPower);
        AL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void gripper(double grippertarget) {

        gripper.setPosition(grippertarget);

    }

    public void armtarget(int height){
        target = height ;
        while (opModeIsActive()) {
            int armPos = AF.getCurrentPosition();

            double error = target - currentTarget;

            double increment = sCurveRampRate * error;
            if (Math.abs(increment) > Math.abs(error)) {
                increment = error;
            }
            currentTarget += increment;

            double pid = controller.calculate(armPos, currentTarget);
            double ff = Math.cos(Math.toRadians(armPos / ticks_in_degree_AA)) * f;

            double rawPower = pid + ff;
            rawPower = Math.max(-maxSpeed, Math.min(maxSpeed, rawPower));

            double powerChange = rawPower - previousPower;
            if (Math.abs(powerChange) > maxAccel) {
                powerChange = Math.signum(powerChange) * maxAccel;
            }

            double finalPower = previousPower + powerChange;
            previousPower = finalPower;

            AF.setPower(finalPower);
            AB.setPower(finalPower);
        }


    }


    public void wrist(double wristLTarget, double wristRTarget) {
        wristL.setPosition(wristLTarget);
        wristR.setPosition(wristRTarget);
    }

    public void customSleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-6.7,-62.4, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        controller = new PIDController(p, i, d);

        AL = hardwareMap.get(DcMotorEx.class,"AL");

        AF = hardwareMap.get(DcMotorEx.class,"AF");

        AB = hardwareMap.get(DcMotorEx.class,"AB");

        AL.setDirection(DcMotorSimple.Direction.REVERSE);

        gripper = hardwareMap.servo.get("gripper");

        wristL = hardwareMap.servo.get("wristL");

        wristL.setDirection(Servo.Direction.REVERSE);

        wristR = hardwareMap.servo.get("wristR");

        AL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        AL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (!isStarted() && !isStopRequested()) {
            // Wait for the DS start button to be touched.

           /* if (currentGamepad2.x && !previousGamepad2.x) {
                start_delay = start_delay + 500;
                telemetry.addData("delay", start_delay);
                telemetry.update();
            } */    //시작 딜레이, 한번 누를때마다 0.5초



        }

        //customSleep(start_delay);  //시작 딜레이

        // Share the CPU.
        sleep(20);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) // 앞으로 이동하면서 팔을 들어올리고 그리퍼 회전
                //  팔 길이를 늘리고 뒤로 이동 후 일정시간 뒤에 그리퍼 놓기
                .stopAndAdd(()->gripper(0.08))
                .stopAndAdd(()->armtarget(targetCUP))
                .waitSeconds(1)
                .stopAndAdd(()->wrist(wristUP,wristUP))
                .waitSeconds(1)
                .lineToY(-24)
                .waitSeconds(0.5)
                .stopAndAdd(()->ALadjust(1,1400))
                .waitSeconds(0.5)
                .lineToY(-36)
                .waitSeconds(0.5)
                .stopAndAdd(()->gripper(0.31));


        Action trajectoryActionCloseOut1 = tab1.fresh()
                .build();

        Action trajectoryAction1;
        trajectoryAction1 = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction1,
                        trajectoryActionCloseOut1
                )
        );

        while (opModeIsActive()) {
            int armPos = AF.getCurrentPosition();

            double error = target - currentTarget;

            double increment = sCurveRampRate * error;
            if (Math.abs(increment) > Math.abs(error)) {
                increment = error;
            }
            currentTarget += increment;

            double pid = controller.calculate(armPos, currentTarget);
            double ff = Math.cos(Math.toRadians(armPos / ticks_in_degree_AA)) * f;

            double rawPower = pid + ff;
            rawPower = Math.max(-maxSpeed, Math.min(maxSpeed, rawPower));

            double powerChange = rawPower - previousPower;
            if (Math.abs(powerChange) > maxAccel) {
                powerChange = Math.signum(powerChange) * maxAccel;
            }

            double finalPower = previousPower + powerChange;
            previousPower = finalPower;

            AF.setPower(finalPower);
            AB.setPower(finalPower);
        }

    }



}