package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="MacanumDrive", group="Driving")
public class Macanum extends LinearOpMode {      //Creates a TeleOp class called Macanum; used for driver-controlled period

    DcMotor zero;
    DcMotor one;
    DcMotor two;
    DcMotor three;

    BNO055IMU dab; //Defines motors and gyroscope

    @Override
    public void runOpMode() //Creates a method called runOpMode
    {

        dab = hardwareMap.get(BNO055IMU.class, "imu"); //Maps the gyroscope to the name "imu" in the phone
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
        parameters.loggingEnabled = false;

        dab.initialize(parameters); //Sets values and initalizes the gyroscope
        while(!dab.isGyroCalibrated() && !isStarted() && !isStopRequested()) {
            sleep(50);
        } //Waits until the gyroscope is calibrated and started

        telemetry.addLine("PIZZA IS READY"); // Adds a line on the phone saying the "Pizza is ready"
        telemetry.update();

        zero = hardwareMap.dcMotor.get("Ella-x"); // Sets the motors to a hardware map with different names (ella/cole x/y)
        one = hardwareMap.dcMotor.get("Cole-x");
        two = hardwareMap.dcMotor.get("Ella-y");
        three = hardwareMap.dcMotor.get("Cole-y");

        MediaPlayer chezbob = MediaPlayer.create(hardwareMap.appContext, R.raw.cantinasong); //Plays music song that sounds like "Give up" but is really called "chezbob"
        MediaPlayer hypesong = MediaPlayer.create(hardwareMap.appContext,R.raw.istanbul);

        DcMotor lift = hardwareMap.dcMotor.get("lifty"); //    Maps the lift

        DcMotor intake = hardwareMap.dcMotor.get("IntakeArm");
        DcMotor spindle = hardwareMap.dcMotor.get("spindley");

        CRServo spinner = hardwareMap.crservo.get("intake");

        zero.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        three.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        zero.setDirection(DcMotor.Direction.FORWARD);
        one.setDirection(DcMotor.Direction.REVERSE);
        two.setDirection(DcMotor.Direction.FORWARD);
        three.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        spindle.setDirection(DcMotor.Direction.FORWARD);
        spinner.setDirection(CRServo.Direction.FORWARD);

        zero.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        chezbob.start();

        while(opModeIsActive())
        {
            Vector inputVector = new Vector(gamepad1.right_stick_x, -gamepad1.right_stick_y);
            telemetry.addData( "x0",inputVector.x);
            telemetry.addData("y0",inputVector.y);
            Orientation angles = dab.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            inputVector.rotate(-(Math.PI/4));

            zero.setPower(inputVector.x+gamepad1.left_stick_x);
            one.setPower(inputVector.y-gamepad1.left_stick_x);
            two.setPower(inputVector.y+gamepad1.left_stick_x);
            three.setPower(inputVector.x-gamepad1.left_stick_x);

            telemetry.addData("x",inputVector.x);
            telemetry.addData("y",inputVector.y);

            lift.setPower(gamepad2.right_stick_y);

            telemetry.addData("a",gamepad2.a);

            if (gamepad2.a) {
                spindle.setPower(0.7);
            }
            else if(gamepad2.b) {
                spindle.setPower(-0.7);
            }
            else  {
                spindle.setPower(0);

            }
            if(gamepad2.y) {
                spinner.setPower(0.7);
            }
            else {
                spinner.setPower(0);
            }
            if(gamepad2.x) {
                chezbob.stop();
                hypesong.start();
            }
            intake.setPower(gamepad2.left_stick_y);
            telemetry.update();
        }

        chezbob.stop();
        hypesong.stop();
    }
    public void turnPID(double angle) {
        Orientation angles1 = dab.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double prevAngle = angles1.firstAngle;

        double kp = 0.02766136770696903;
        double ki = 0;
        double kd = 0.019047809331823373;

        double target = 70;

        double elapsedTime = 1;
        double errorSum = 0;

        double previousError = 0;
        double error;

        while(opModeIsActive()) {

            kp += -gamepad1.right_stick_y/10000;
            kd += -gamepad1.left_stick_y/10000;
            ki += -gamepad2.right_stick_y/Math.pow(10000,2);

            double startTime = System.currentTimeMillis();
            Orientation angles = dab.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = target - angles.firstAngle;

            double p = kp*error;
            double i = (ki*error+errorSum)*elapsedTime;
            double d = -kd*(angles.firstAngle-prevAngle)/elapsedTime;

            double pid = p+i+d;

            //Counter Clockwise
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
