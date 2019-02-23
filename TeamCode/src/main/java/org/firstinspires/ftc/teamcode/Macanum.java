package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name="MacanumDrive", group="Driving")
public class Macanum extends LinearOpMode{

    @Override
    public void runOpMode()
    {

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

        telemetry.addLine("PIZZA IS READY");
        telemetry.update();

        DcMotor zero = hardwareMap.dcMotor.get("Ella-x");
        DcMotor one = hardwareMap.dcMotor.get("Cole-x");
        DcMotor two = hardwareMap.dcMotor.get("Ella-y");
        DcMotor three = hardwareMap.dcMotor.get("Cole-y");

        DcMotor intake = hardwareMap.dcMotor.get("IntakeArm");
        DcMotor spindle = hardwareMap.dcMotor.get("spindley");
        CRServo spinner = hardwareMap.crservo.get("intake");

        MediaPlayer chezbob = MediaPlayer.create(hardwareMap.appContext, R.raw.chezbob);

        DcMotor lift = hardwareMap.dcMotor.get("lifty");

        zero.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        three.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        zero.setDirection(DcMotor.Direction.FORWARD);
        one.setDirection(DcMotor.Direction.REVERSE);
        two.setDirection(DcMotor.Direction.FORWARD);
        three.setDirection(DcMotor.Direction.REVERSE);

        lift.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        chezbob.start();

        while(opModeIsActive())
        {
            Vector inputVector = new Vector(gamepad1.right_stick_x, -gamepad1.right_stick_y);
            telemetry.addData( "x0",inputVector.x);
            telemetry.addData("y0",inputVector.y);
            Orientation angles = dab.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            inputVector.rotate(-(Math.PI/4 + angles.firstAngle));

            zero.setPower(inputVector.x+gamepad1.left_stick_x);
            one.setPower(inputVector.y-gamepad1.left_stick_x);
            two.setPower(inputVector.y+gamepad1.left_stick_x);
            three.setPower(inputVector.x-gamepad1.left_stick_x);

            telemetry.addData("x",inputVector.x);
            telemetry.addData("y",inputVector.y);
            telemetry.update();

            lift.setPower(gamepad2.right_stick_y);

        }

        chezbob.stop();
    }
}
