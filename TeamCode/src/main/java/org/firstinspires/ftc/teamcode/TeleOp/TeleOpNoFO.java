package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TeleOpNoFO extends OpMode {

    DcMotor motor3;
    DcMotor motor2;
    DcMotor motor1;
    DcMotor motor0;
    Servo servo0;
    CRServo servo1;
    IMU imu;
    //TouchSensor touch0;
    double spinRate;
    double left_angle;
    double right_angle;
    double idealYaw;
    int spinMotor = 3;
    double servoPosition = 0;
    double CPR = 54.8 * 28;

    double speed = 0.75;

    int rotations = 0;

    private volatile boolean rotationThreadActive = false;


    boolean toggleClaw = false;

    public void init(){
        //touch init
        //touch0 = hardwareMap.touchSensor.get("touch0");

        //servo init
        servo0 = hardwareMap.servo.get("servo0");
        servo1 = hardwareMap.crservo.get("servo1");

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
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setTargetPosition(0);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //motor3.setPower(1);


    }

    public double getRobotAngle(){
        AngleUnit degrees = AngleUnit.DEGREES;
        return imu.getRobotYawPitchRollAngles().getYaw(degrees);
    }


    public void moveToAngle(double targetAngle) {
        // Specify a proportional constant for control
        double proportionalConstant = 0.02;

        // Allow for a small margin of error
        double angleErrorMargin = 5.0;

        while (Math.abs(getAngleError(targetAngle)) > angleErrorMargin) {
            double power = getAngleError(targetAngle) * proportionalConstant;

            // Apply power to the appropriate motor(s) based on spinMotor value
            if (spinMotor == 0) {
                motor0.setPower(power);
            } else if (spinMotor == 1) {
                motor1.setPower(power);
            } else if (spinMotor == 2) {
                motor2.setPower(power);
            } else if (spinMotor == 3) {
                motor0.setPower(power);
                motor1.setPower(power);
                motor2.setPower(power);
            }

            // Add a delay to avoid excessive updates
            try {
                Thread.sleep(10); // Adjust the delay time as needed
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        // Stop the motors after reaching the desired angle
        stopMotors();
    }

    // Helper method to calculate the angle error
    private double getAngleError(double targetAngle) {
        double currentAngle = getRobotAngle();
        double angleDifference = targetAngle - currentAngle;

        // Ensure the angle difference is between -180 and 180 degrees
        if (angleDifference > 180.0) {
            angleDifference -= 360.0;
        } else if (angleDifference < -180.0) {
            angleDifference += 360.0;
        }

        return angleDifference;
    }

    // Helper method to stop all motors
    private void stopMotors() {
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
    }



    public void moveRobot(double moveAngleDegrees, double speed, double rotationDegrees) {
        double moveAngleRadians = Math.toRadians(moveAngleDegrees); // Convert move angle to radians
        double robotAngle = Math.atan2(Math.cos(moveAngleRadians), Math.sin(moveAngleRadians) * Math.sqrt(3)); // Angle of the robot

        double motor0Power = -Math.sin(robotAngle + Math.PI / 3); // Motor 0 power
        double motor1Power = -Math.sin(robotAngle - Math.PI / 3); // Motor 1 power
        double motor2Power = Math.sin(robotAngle); // Motor 2 power

        // Rotate to the desired angle before moving
        moveToAngle(rotationDegrees);

        stopMotors();

        // Set motor powers for movement
        motor0.setPower(motor0Power * speed);
        motor1.setPower(motor1Power * speed);
        motor2.setPower(motor2Power * speed);
    }



    public void loop(){
        left_angle = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) * (180/ Math.PI) + 180;
        
//        if(gamepad1.right_stick_x > 0) spinRate = gamepad1.right_stick_x * gamepad1.right_stick_x;
//        else spinRate = -gamepad1.right_stick_x * gamepad1.right_stick_x;
        right_angle = Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y) * (180/ Math.PI) + 180;
        
        telemetry.addData("Controller", String.valueOf(left_angle));
        telemetry.addData("spinRate", String.valueOf(spinRate));
        telemetry.addData("servoPosition", String.valueOf(servoPosition));
        telemetry.addData("servoPosition/10000", String.valueOf(servoPosition/500));
        if(motor3.getCurrentPosition() == 28){
            rotations++;
        }
        telemetry.addData("arm motor rotation", String.valueOf((rotations*28+motor3.getCurrentPosition())/CPR));
        telemetry.update();

        speed = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));

       if(gamepad1.x) speed *= 0.5;
//        else speed = 0.75;
        if(left_angle < 60 && left_angle > 0 && left_angle != 180){
//            motor0.setPower(speed);
//            motor1.setPower(-speed);
//            motor2.setPower(spinRate);
            spinMotor = 2;
        }
        else if(left_angle < 120 && left_angle > 60 && left_angle != 180){
//            motor1.setPower(speed);
//            motor2.setPower(-speed);
//            motor0.setPower(spinRate);
            spinMotor = 0;
        }
        else if(left_angle < 180 && left_angle > 120 && left_angle != 180){
//            motor0.setPower(-speed);
//            motor2.setPower(speed);
//            motor1.setPower(spinRate);
            spinMotor = 1;
        }
        else if(left_angle < 240 && left_angle > 180 && left_angle != 180){
//            motor0.setPower(-speed);
//            motor1.setPower(speed);
//            motor2.setPower(spinRate);
            spinMotor = 2;
        }
        else if(left_angle < 300 && left_angle > 240 && left_angle != 180){
//            motor1.setPower(-speed);
//            motor2.setPower(speed);
//            motor0.setPower(spinRate);
            spinMotor = 0;
        }
        else if(left_angle <= 360 && left_angle > 300 && left_angle != 180){
//            motor0.setPower(speed);
//            motor2.setPower(-speed);
//            motor1.setPower(spinRate);
            spinMotor = 1;
        }
        else{
//            motor0.setPower(spinRate);
//            motor1.setPower(spinRate);
//            motor2.setPower(spinRate);
            spinMotor = 3;
        }

        if(gamepad1.y){
            stopMotors();
        }

        moveRobot(left_angle, speed, right_angle);

        if(gamepad1.left_trigger >0){
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setPower(0.25);
        }
        else if(gamepad1.left_bumper){
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setPower(-0.25);
        }
        else{
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setPower(0);
        }

        if(gamepad1.right_trigger > 0 && servoPosition <= 499){
            servoPosition++;
            servo0.setPosition(servoPosition/500);
        }
        else if(gamepad1.right_bumper && servoPosition >= 1){
            servoPosition--;
            servo0.setPosition(servoPosition/500);
        }
        else if(servoPosition >= 0 && servoPosition <= 500) {
            servo0.setPosition(servoPosition / 500);
        }
        else{
            servoPosition = 0;
            servo0.setPosition(servoPosition/500);
        }



//        if(gamepad1.x){
//            motor3.setTargetPosition((int)(CPR*0.25));
//            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        else if(gamepad1.y){
//            motor3.setTargetPosition((int)(CPR*0.4));
//            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        else if(gamepad1.dpad_left){
//            motor3.setTargetPosition(0);
//            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }

        //toggle claw servo (servo1)
        if(gamepad1.a){
            servo1.setPower(1);
        }
        else if (gamepad1.b){
            servo1.setPower(-1);
        }
        else{
            servo1.setPower(0);
        }
    }
}
