package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutonomousTest extends OpMode {

    DcMotor motor3;
    DcMotor exMotor0;

    double CPR = 54.8 * 28;

    public void init(){
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setTargetPosition(0);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        exMotor0 = hardwareMap.dcMotor.get("exMotor0");
        exMotor0.setPower(1);
    }
    public void loop(){
        int position = motor3.getCurrentPosition();
        double revolutions = position/CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;
        motor3.setTargetPosition(1500);
        motor3.setPower(1);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
