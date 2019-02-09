package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="PIDTuner",group = "")
public class PIDTuner extends LinearOpMode { //This names the class and defines it as an OpMode (Driver Controlled)

    @Override //Lets us make our own program, overriding the already set LinearOpMode function
    public void runOpMode() {
        //Code to implement and run the gyroscope
        BNO055IMU dab = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
        parameters.loggingEnabled = false;

        dab.initialize(parameters);
        while(!dab.isGyroCalibrated() && !isStarted() && !isStopRequested()) {
            sleep(50);
        }
        //Adds a line to the phone that says "pizza is ready", so we know its initialized
        telemetry.addLine("PIZZA IS READY");
        telemetry.update();
        //Initializes all the four motors, to different names
        DcMotor zero = hardwareMap.dcMotor.get("Ella-x");
        DcMotor one = hardwareMap.dcMotor.get("Cole-x");
        DcMotor two = hardwareMap.dcMotor.get("Ella-y");
        DcMotor three = hardwareMap.dcMotor.get("Cole-y");

        //Below line is commented out, but sets up music on our robot
        //MediaPlayer chezbob = MediaPlayer.create(hardwareMap.appContext, R.raw.chezbob);

        //DcMotor lift = hardwareMap.dcMotor.get("lifty");

        //Sets all the motors to be run using encoders, which counts the values
        zero.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        three.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Sets all the drive motors to proper directions so they all go the same way
        zero.setDirection(DcMotor.Direction.FORWARD);
        one.setDirection(DcMotor.Direction.REVERSE);
        two.setDirection(DcMotor.Direction.FORWARD);
        three.setDirection(DcMotor.Direction.REVERSE);

        //Sets all the drive motors to brake after being set to no power
        zero.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //lift.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        //PID Tuner values that make the robot be able to turn to an angle decently fast without too much oscillation
        double kp = 0.02766136770696903;
        double ki = 0;
        double kd = 0.019047809331823373;

        double target = 70;

        double elapsedTime = 1;
        double errorSum = 0;

        double previousError = 0;
        double error;

        Orientation angles1 = dab.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double prevAngle = angles1.firstAngle;

        // Makes a function that sets the PID values to go up or down based on the gamepad, so you can change the values through the controller without reuploading code
        while(opModeIsActive()) {

<<<<<<< HEAD
            kp += -gamepad1.right_stick_y/10000;
            kd += -gamepad1.left_stick_y/10000;
            ki += -gamepad2.right_stick_y/Math.pow(10000,2);
=======
            kp += -gamepad1.right_stick_y / (gamepad1.right_stick_y + 150000);
            kd += -gamepad1.left_stick_y / (gamepad1.left_stick_y + 150000);
            ki += -gamepad2.right_stick_y / (gamepad2.right_stick_y + 150000);
>>>>>>> ba5d16dc1e6db5ecb62f047b420289252c9fd5d1

            double startTime = System.currentTimeMillis();
            Orientation angles = dab.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = target - angles.firstAngle;

            double p = kp*error;
            double i = (ki*error+errorSum)*elapsedTime;
            double d = -kd*(angles.firstAngle-prevAngle)/elapsedTime;

            double pid = p+i+d;

            //Counter Clockwise; sets all the PID values
            if(Math.abs(error) > 0.5) {
                zero.setPower(-pid);
                one.setPower(pid);
                two.setPower(-pid);
                three.setPower(pid);
            }
            else {
                zero.setPower(0);
                one.setPower(0);
                two.setPower(0);
                three.setPower(0);
            }
            errorSum += ki*error;

            //Puts values on the phone that display the Roll, Pitch, Yaw, and the PID values
            telemetry.addData("Roll",dab.getAngularOrientation().secondAngle);
            telemetry.addData("Pitch",dab.getAngularOrientation().thirdAngle);
            telemetry.addData("Yeet",dab.getAngularOrientation().firstAngle);

            telemetry.addData("kp",kp);
            telemetry.addData("ki",ki);
            telemetry.addData("kd",kd);

            telemetry.update();
            previousError = error;
            errorSum=Range.clip(errorSum,-37,37); //because why not
            elapsedTime = System.currentTimeMillis()-startTime;
            prevAngle = angles.firstAngle;
        }
    }
}
