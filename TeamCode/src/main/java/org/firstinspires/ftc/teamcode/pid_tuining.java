package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config

@TeleOp
public class pid_tuining extends OpMode{
    private PIDController controller_AA;

    public static double p_AA = 0, i_AA = 0, d_AA = 0;
    public static double f_AA = 0;

    public static int target_AA = 0;

    private final double ticks_in_degree_AA = 700 / 180.0;


    private PIDController controller_AL;

    public static double p_AL = 0, i_AL = 0, d_AL = 0;
    public static double f_AL = 0;

    public static int target_AL = 0;

    private final double ticks_in_degree_AL = 700 / 180.0;


    private DcMotorEx AL;
    private DcMotorEx AA;

    @Override
    public void init() {
        controller_AA = new PIDController(p_AA, i_AA, d_AA);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller_AL = new PIDController(p_AL, i_AL, d_AL);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        AL = hardwareMap.get(DcMotorEx.class, "AL");
        AA = hardwareMap.get(DcMotorEx.class, "AA");

        AL.setDirection(DcMotorSimple.Direction.REVERSE);
        AA.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        controller_AA.setPID(p_AA, i_AA, d_AA);
        int armPos_AA = AA.getCurrentPosition();
        double pid_AA = controller_AA.calculate(armPos_AA, target_AA);
        double ff_AA = Math.cos(Math.toRadians(target_AA / ticks_in_degree_AA)) * f_AA;

        double power_AA = pid_AA + ff_AA;


        controller_AL.setPID(p_AL, i_AL, d_AL);
        int armPos_AL = AL.getCurrentPosition();
        double pid_AL = controller_AL.calculate(armPos_AL, target_AL);
        double ff_AL = Math.cos(Math.toRadians(target_AL / ticks_in_degree_AL)) * f_AL;

        double power_AL = pid_AL + ff_AL;

        AL.setPower(power_AL);
        AA.setPower(power_AA);

        telemetry.addData("pos_AA ", armPos_AA);
        telemetry.addData("pos_AL ", armPos_AL);
        telemetry.addData("target_AA ", target_AA);
        telemetry.addData("target_AL ", target_AL);
        telemetry.update();
    }
}