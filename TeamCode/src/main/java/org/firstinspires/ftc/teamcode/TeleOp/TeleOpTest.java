package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TeleOpTest extends OpMode {

    DcMotor motor3;
    DcMotor motor2;
    DcMotor motor1;
    DcMotor motor0;
    Servo servo0;
    IMU imu;
    //TouchSensor touch0;
    double spinRate;
    double left_angle;
    double right_angle;
    double idealYaw;
    int spinMotor = 3;


    double speed;

    public void init(){
        //touch init
        //touch0 = hardwareMap.touchSensor.get("touch0");

        //servo init
        servo0 = hardwareMap.servo.get("servo0");

        imu = hardwareMap.get(IMU.class, "imu");

        //imu init
        imu.resetYaw();

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

    public double getRobotAngle(){
        AngleUnit degrees = AngleUnit.DEGREES;
        return imu.getRobotYawPitchRollAngles().getYaw(degrees);
    }

    public void moveToAngleHelper(double spinRate, double angle){
        if(spinMotor == 0){
            motor0.setPower(spinRate);
            motor1.setPower(spinRate);
            motor2.setPower(spinRate);
        }
        else if(spinMotor == 1){
            motor0.setPower(spinRate);
            motor1.setPower(spinRate);
            motor2.setPower(spinRate);
        }
        else if(spinMotor == 2){
            motor0.setPower(spinRate);
            motor1.setPower(spinRate);
            motor2.setPower(spinRate);
        }
        else if(spinMotor == 3){
            motor0.setPower(spinRate);
            motor1.setPower(spinRate);
            motor2.setPower(spinRate);
        }
    }

    public void moveToAngle(double angle){
        double currentAngle = getRobotAngle();
        double angleDifference = angle - currentAngle;
        double spinRate = angleDifference/180;

        while(getRobotAngle() != angle){
            if(getRobotAngle() > angle){
                spinRate = -spinRate;
            }
            else if(getRobotAngle() < angle){
                spinRate = Math.abs(spinRate);
            }
            moveToAngleHelper(spinRate, angle);
        }
        spinRate=0;
        moveToAngleHelper(spinRate, angle);
    }


    public void loop(){
        left_angle = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) * (180/ Math.PI) + 180;
        right_angle = Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y) * (180/ Math.PI) + 180;
        speed = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        spinRate = (double)gamepad1.right_stick_x;
        telemetry.addData("Controller", String.valueOf(left_angle));
        telemetry.addData("spinRate", String.valueOf(spinRate));
        telemetry.update();

        if(left_angle < 60/* - getRobotAngle() */&& left_angle > 0/* - getRobotAngle() */&& left_angle != 180){
            motor0.setPower(speed);
            motor1.setPower(-speed);
            //motor2.setPower(spinRate);
            spinMotor = 2;
        }
        else if(left_angle < 120/* - getRobotAngle() */&& left_angle > 60/* - getRobotAngle() */&& left_angle != 180){
            motor1.setPower(speed);
            motor2.setPower(-speed);
            //motor0.setPower(spinRate);
            spinMotor = 0;
        }
        else if(left_angle < 180/* - getRobotAngle() */&& left_angle > 120/* - getRobotAngle() */&& left_angle != 180){
            motor0.setPower(-speed);
            motor2.setPower(speed);
            //motor1.setPower(spinRate);
            spinMotor = 1;
        }
        else if(left_angle < 240/* - getRobotAngle() */&& left_angle > 180/* - getRobotAngle() */&& left_angle != 180){
            motor0.setPower(-speed);
            motor1.setPower(speed);
            //motor2.setPower(spinRate);
            spinMotor = 2;
        }
        else if(left_angle < 300/* - getRobotAngle() */&& left_angle > 240/* - getRobotAngle() */&& left_angle != 180){
            motor1.setPower(-speed);
            motor2.setPower(speed);
//            motor0.setPower(spinRate);
            spinMotor = 0;
        }
        else if(left_angle <= 360/* - getRobotAngle() */&& left_angle > 300/* - getRobotAngle() */&& left_angle != 180){
            motor0.setPower(speed);
            motor2.setPower(-speed);
//            motor1.setPower(spinRate);
            spinMotor = 1;
        }
        else{
//            motor0.setPower(spinRate);
//            motor1.setPower(spinRate);
//            motor2.setPower(spinRate);
            spinMotor = 3;
        }
        if(right_angle != 180){
            moveToAngle(right_angle);
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
