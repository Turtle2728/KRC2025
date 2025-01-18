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
public class PID_TEST extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0; // PID 게인
    public static double f = 0; // 중력 보상 게인
    public static double maxSpeed = 0.5; // 최대 속도 제한
    public static double maxAccel = 0.02; // 최대 가속도 제한

    public static int target = 0; // 사용자가 입력하는 목표 위치
    public static double sCurveRampRate = 0.05; // S-curve 변화율 (0 ~ 1)

    private final double ticks_in_degree = 800 / 180.0; // 1도당 틱 수 계산
    private DcMotorEx AF;
    private DcMotorEx AB;


    private double currentTarget = 0; // 점진적으로 변경되는 목표값
    private double previousPower = 0; // 이전 루프의 출력값 저장

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AF = hardwareMap.get(DcMotorEx.class, "AF");
        AB = hardwareMap.get(DcMotorEx.class, "AB");



        controller.setPID(p, i, d); // PID 초기화
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d); // PID 값 업데이트

        // S-curve 프로파일로 목표값 점진적 변화
        double error = target - currentTarget;
        double increment = sCurveRampRate * error; // 변화율에 따른 점진적 증가값
        if (Math.abs(increment) > Math.abs(error)) {
            increment = error; // 목표에 가까워지면 정확히 도달
        }
        currentTarget += increment;

        int armPos = AF.getCurrentPosition(); // 현재 모터 위치
        double pid = controller.calculate(armPos, currentTarget); // PID 계산
        double ff = Math.cos(Math.toRadians(armPos / ticks_in_degree)) * f; // 중력 보상 계산

        double rawPower = pid + ff; // PID + 중력 보상
        rawPower = Math.max(-maxSpeed, Math.min(maxSpeed, rawPower)); // 속도 제한 적용

        // 가속도 제한 적용
        double powerChange = rawPower - previousPower;
        if (Math.abs(powerChange) > maxAccel) {
            powerChange = Math.signum(powerChange) * maxAccel; // 가속도 제한
        }

        double finalPower = previousPower + powerChange;
        previousPower = finalPower; // 이전 출력값 업데이트

        // 모터 출력 설정
        AF.setPower(finalPower);
        AB.setPower(finalPower);


        // Telemetry로 디버깅 정보 출력
        telemetry.addData("Current Target", currentTarget);
        telemetry.addData("Position", armPos);
        telemetry.addData("Target", target);
        telemetry.addData("Power", finalPower);
        telemetry.update();
    }
}
