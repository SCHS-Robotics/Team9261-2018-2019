package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Chief Red Jacket", group="banana")
public class ChiefRedJacket extends LinearOpMode {
    Servo stab;
    @Override
    public void runOpMode(){
        stab = hardwareMap.servo.get("stab");
        stab.setDirection(Servo.Direction.FORWARD);
        stab.setPosition(1);
        waitForStart();
        raskolnikov();
        sleep(10000);
    }
    public void raskolnikov() {
        stab.setPosition(-1);
        sleep(3000);
        stab.setPosition(1);
    }
}
