package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="TRINITY", group="HOLEY")
public class HoleyTrinity extends LinearOpMode {

    DcMotor uno;
    DcMotor dos;
    DcMotor tres;
    DcMotor cuatro;

    @Override
    public void runOpMode(){

        uno = hardwareMap.dcMotor.get("Ella-x");
        uno.setDirection(DcMotor.Direction.FORWARD);
        dos = hardwareMap.dcMotor.get("Cole-x");
        dos.setDirection(DcMotor.Direction.FORWARD);
        tres = hardwareMap.dcMotor.get("Ella-y");
        tres.setDirection(DcMotor.Direction.REVERSE);
        cuatro = hardwareMap.dcMotor.get("Cole-y");
        cuatro.setDirection(DcMotor.Direction.REVERSE);

        uno.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tres.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cuatro.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        uno.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dos.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tres.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cuatro.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        redStaff(7000,1);
        redStaff(6000,-0.5);

    }

    public void redStaff(int targetPosition, double power) {
        while(opModeIsActive() && (uno.getCurrentPosition() < targetPosition || dos.getCurrentPosition() < targetPosition || tres.getCurrentPosition() < targetPosition || cuatro.getCurrentPosition() < targetPosition)) {
            uno.setPower(power);
            dos.setPower(power);
            tres.setPower(power);
            cuatro.setPower(power);
        }
        uno.setPower(0);
        dos.setPower(0);
        tres.setPower(0);
        cuatro.setPower(0);
    }
}
