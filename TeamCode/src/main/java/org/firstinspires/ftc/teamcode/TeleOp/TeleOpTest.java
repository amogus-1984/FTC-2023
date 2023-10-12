package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class TeleOpTest extends OpMode {

    DcMotor motor3;
    DcMotor motor2;
    DcMotor motor1;
    DcMotor motor0;
    Servo servo0;
    //TouchSensor touch0;
    double spinRate;
    double left_angle;

    public void init(){
        //touch init
        //touch0 = hardwareMap.touchSensor.get("touch0");

        //servo init
        servo0 = hardwareMap.servo.get("servo0");

        //motor init
        motor0 = hardwareMap.dcMotor.get("motor0");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");

        motor0.setDirection(DcMotor.Direction.FORWARD);
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);

        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor3.setPower(1);


    }

    public void loop(){
        left_angle = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) * (180/ Math.PI) + 180;
        spinRate = (double)gamepad1.right_stick_x;
        telemetry.addData("Controller", String.valueOf(left_angle));
        telemetry.addData("spinRate", String.valueOf(spinRate));
        telemetry.update();

        if(left_angle < 60 && left_angle > 0 && left_angle != 180){
            motor0.setPower(0.75);
            motor1.setPower(-0.75);
            motor2.setPower(spinRate);
        }
        else if(left_angle < 120 && left_angle > 60 && left_angle != 180){
            motor1.setPower(0.75);
            motor2.setPower(-0.75);
            motor0.setPower(spinRate);
        }
        else if(left_angle <= 180 && left_angle > 120 && left_angle != 180){
            motor0.setPower(-0.75);
            motor2.setPower(0.75);
            motor1.setPower(spinRate);
        }
        else if(left_angle < 240 && left_angle > 180 && left_angle != 180){
            motor0.setPower(-0.75);
            motor1.setPower(0.75);
            motor2.setPower(spinRate);
        }
        else if(left_angle < 300 && left_angle > 240 && left_angle != 180){
            motor1.setPower(-0.75);
            motor2.setPower(0.75);
            motor0.setPower(spinRate);
        }
        else if(left_angle <= 360 && left_angle > 300 && left_angle != 180){
            motor0.setPower(0.75);
            motor2.setPower(-0.75);
            motor1.setPower(spinRate);
        }
        else{
            motor0.setPower(spinRate);
            motor1.setPower(spinRate);
            motor2.setPower(spinRate);
        }



        if (gamepad1.dpad_down) {
            servo0.setPosition(0);
        } else if (gamepad1.dpad_left) {
            servo0.setPosition(0.33);
        } else if (gamepad1.dpad_up) {
            servo0.setPosition(0.66);
        } else if (gamepad1.dpad_right) {
            servo0.setPosition(1);
        }
    }
}
