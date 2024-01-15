package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class AutonomousTest extends OpMode {

    DcMotor motor3;
    DcMotor motor2;
    DcMotor motor1;
    DcMotor motor0;
    IMU imu;
    int spinMotor = 3;
    double CPR = 54.8 * 28;



    public void init(){
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

        moveRobot(0, 1, 0);
        try{
            Thread.sleep(1000);
        }
        catch(InterruptedException e){
            e.printStackTrace();
        }
        stopMotors();
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

    }
}
