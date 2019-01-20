package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.bioinspired.Retina;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

@TeleOp(name="Some Linear OpMode", group="OpMode")
public class testing extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2{

    Retina retina;
    @Override
    public void runOpMode() {
        // Initialize hardware code goes here

        waitForStart();

        startOpenCV(this);

        while (opModeIsActive()) {
            // Do stuff
        }

        stopOpenCV();
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        retina = Retina.create(new Size(320,240));
        retina.setup();
        retina.clearBuffers();
    }

    @Override
    public void onCameraViewStopped() {
        retina.clear();
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        // This is where the magic will happen. inputFrame has all the data for each camera frame.
        Mat input = inputFrame.rgba().clone();
        Imgproc.cvtColor(input,input, Imgproc.COLOR_RGBA2RGB);
        Imgproc.resize(input,input,new Size(320,240));
        input.convertTo(input, CvType.CV_32F);
        inputFrame.rgba().release();
        inputFrame.gray().release();
        Mat toneMap = new Mat();
        retina.applyFastToneMapping(input,toneMap);
        toneMap.convertTo(toneMap, CvType.CV_8U);
        Imgproc.resize(toneMap,toneMap,new Size(1280,720));

        return toneMap;
    }

    public void startOpenCV(CameraBridgeViewBase.CvCameraViewListener2 cameraViewListener) {
        FtcRobotControllerActivity.turnOnCameraView.obtainMessage(1, cameraViewListener).sendToTarget();
    }

    public void stopOpenCV() {
        FtcRobotControllerActivity.turnOffCameraView.obtainMessage().sendToTarget();
    }
}