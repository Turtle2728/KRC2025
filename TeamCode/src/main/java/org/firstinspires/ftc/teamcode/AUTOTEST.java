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
@Autonomous(name = "AUTOTEST", group = "2025KRC_AUTO")
public class AUTOTEST extends LinearOpMode {

    private final double wristUP = 0.28;

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

    public void AAadjust(double AAPower, int AATarget) {
        AF.setTargetPosition(AATarget);
        AF.setPower(AAPower);
        AF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        AB.setTargetPosition(AATarget);
        AB.setPower(AAPower);
        AB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void gripper(double grippertarget) {

        gripper.setPosition(grippertarget);

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

        AF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        AF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        AB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        AB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
                .stopAndAdd(()->AAadjust(1,150))
                .waitSeconds(1)
                .stopAndAdd(()->wrist(wristUP,wristUP))
                .waitSeconds(1)
                .lineToY(-36)
                .waitSeconds(0.5)
                .stopAndAdd(()->ALadjust(1,1400))
                .waitSeconds(2)
                .lineToY(-48)
                .waitSeconds(2)
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

    }

}