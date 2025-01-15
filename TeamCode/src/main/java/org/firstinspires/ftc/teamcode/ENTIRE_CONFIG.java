package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config

@TeleOp
public class ENTIRE_CONFIG extends OpMode{

    public static double POS_wristL = 0;
    public static double POS_wristR = 0;

    public static double POS_gripper = 0.2;
    //public static double POS_H_angleR = 0.5;


    //private Servo V_wristR;

    private Servo wristL;
    private Servo wristR;
    private Servo gripper;
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




//
        //Servo V_wristR = hardwareMap.servo.get("V_wristR"); //Bucket Wrist right Servo
        wristL = hardwareMap.servo.get("wristL"); //Bucket Wrist left Servo

        wristR = hardwareMap.servo.get("wristR"); // Ground Gripper right Servo

        gripper = hardwareMap.servo.get("gripper");


    }

    @Override
    public void loop() {


        wristL.setPosition(POS_wristL);
        wristR.setPosition(POS_wristR);
        gripper.setPosition(POS_gripper);

        wristL.setDirection(Servo.Direction.REVERSE);

        telemetry.update();
    }
}