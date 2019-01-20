package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="MacanumDrive", group="Driving")
public class Macanum extends LinearOpMode{
    @Override
    public void runOpMode()
    {
        DcMotor zero = hardwareMap.dcMotor.get("Ella-x");
        DcMotor one = hardwareMap.dcMotor.get("Cole-x");
        DcMotor two = hardwareMap.dcMotor.get("Ella-y");
        DcMotor three = hardwareMap.dcMotor.get("Cole-y");

        MediaPlayer chezbob = MediaPlayer.create(hardwareMap.appContext, R.raw.chezbob);

        //DcMotor lift = hardwareMap.dcMotor.get("lifty");

        zero.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        three.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        zero.setDirection(DcMotor.Direction.FORWARD);
        one.setDirection(DcMotor.Direction.REVERSE);
        two.setDirection(DcMotor.Direction.FORWARD);
        three.setDirection(DcMotor.Direction.REVERSE);

        //lift.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        chezbob.start();

        while(opModeIsActive())
        {
            Vector inputVector = new Vector(gamepad1.right_stick_x, -gamepad1.right_stick_y);
            telemetry.addData("x0",inputVector.x);
            telemetry.addData("y0",inputVector.y);
            inputVector.rotate(-Math.PI/4);

            zero.setPower(inputVector.x);
            one.setPower(inputVector.y);
            two.setPower(inputVector.y);
            three.setPower(inputVector.x);

            telemetry.addData("x",inputVector.x);
            telemetry.addData("y",inputVector.y);
            telemetry.update();

            //lift.setPower(gamepad2.right_stick_y);
        }

        chezbob.stop();
    }
}
