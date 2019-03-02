package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.bioinspired.Retina;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="BenHartleeeeee Crater", group="PleaseHelp")
public class EmergencyAutoCrater extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2{

    DcMotor zero;
    DcMotor one;
    DcMotor two;
    DcMotor three;

    boolean stop = false;

    DcMotor lift;

    //Servo stab;

    BNO055IMU imu;

    Point prevCenter = new Point(-1,-1);
    Point blockCenter = new Point(0,0);
    double xMax = 320;

    Retina retina;

    @Override
    public void runOpMode() throws InterruptedException {

        zero = hardwareMap.dcMotor.get("Ella-x");
        one = hardwareMap.dcMotor.get("Cole-x");
        two = hardwareMap.dcMotor.get("Ella-y");
        three = hardwareMap.dcMotor.get("Cole-y");



        //MediaPlayer chezbob = MediaPlayer.create(hardwareMap.appContext, R.raw.chezbob);

        lift = hardwareMap.dcMotor.get("lifty");

        //stab = hardwareMap.servo.get("stab");

        zero.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        three.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        zero.setDirection(DcMotor.Direction.FORWARD);
        one.setDirection(DcMotor.Direction.REVERSE);
        two.setDirection(DcMotor.Direction.FORWARD);
        three.setDirection(DcMotor.Direction.REVERSE);

        //stab.setPosition(1);

        telemetry.addData("Mode", "DONE :)");
        telemetry.update();

        waitForStart();


        deploy();
        initGyro();
        while(opModeIsActive() && !imu.isGyroCalibrated()){
            sleep(50);
        }
        //drive(new Vector(0,-1),200);
        //drive(new Vector(1,0),300);
        /*sleep(1000);
        long similarityTime = System.currentTimeMillis();
        long lostTime = 0;
        while(prevCenter.x != blockCenter.x && opModeIsActive() && lostTime < 100) {
            if(prevCenter.x == blockCenter.x) {
                lostTime = System.currentTimeMillis()-similarityTime;
                telemetry.addLine("Lost cube");
            }
            else {
                similarityTime = System.currentTimeMillis();
            }
            driveNoDist(new Vector(feedback(blockCenter.x,xMax/2.0),-0.5));
        }
        sleep(1000);
        setZero();*/
        //drive(new Vector(0,-1),400);
        //sleep(1000);
        //turnPID(0.7,180,5);
        //stab.setPosition(1);
        turnPID(0.7, -90,1);
        drive(new Vector(0,1),3000);
        sleep(1000);
        //stab.setPosition(-1);
        sleep(1000);
    }


    public void drive(Vector v, double distance) {

        zero.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        two.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        three.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        zero.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        three.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        v.rotate(-Math.PI/4);

        while(Math.abs(zero.getCurrentPosition()) < Math.abs(distance) && Math.abs(one.getCurrentPosition()) < Math.abs(distance) && Math.abs(two.getCurrentPosition()) < Math.abs(distance) && Math.abs(three.getCurrentPosition()) < Math.abs(distance) && opModeIsActive()) {
            zero.setPower(v.x);
            one.setPower(v.y);
            two.setPower(v.y);
            three.setPower(v.x);

        }
        zero.setPower(0);
        one.setPower(0);
        two.setPower(0);
        three.setPower(0);
    }

    public void driveNoDist(Vector v) {
        v.rotate(Math.PI/4);

        zero.setPower(-v.x);
        one.setPower(v.y);
        two.setPower(-v.x);
        three.setPower(v.y);
    }
    public void setZero() {
        zero.setPower(0);
        one.setPower(0);
        two.setPower(0);
        three.setPower(0);
    }

    public void deploy() {
        int initialPosition = lift.getCurrentPosition();
        double targetPosition = -19869; //13 rack teeth down start
        while(lift.getCurrentPosition() > targetPosition+initialPosition && opModeIsActive()) {
            lift.setPower(-1);
        }
        lift.setPower(0);

    }

    public void turnPID(double velocity, double angle, double threshold) throws InterruptedException {
        double elapsedTime = 1;
        double accumulatedError = 0;
        double lastError = getYaw()-angle;
        double kp = 0.02766136770696903;
        double ki = 0;
        double kd = 0.019047809331823373;
        double error = 0;
        while (Math.abs(getYaw() - angle) > threshold && opModeIsActive()) {
            double currentTime = System.currentTimeMillis();
            double sign = Math.abs((angle-getYaw())) <= 360-Math.abs((angle-getYaw())) ? Math.signum(angle-getYaw()) : -Math.signum(angle-getYaw());
            //double error = sign* Math.min(Math.abs(angle-getYaw()),360-Math.abs((angle-getYaw())));
            error = angle%360-getYaw()%360;

            double p = kp * error; //proportional component, bases the change to output request based on the amount of error present
            double i = ki * (error + accumulatedError) * elapsedTime;//integral component, bases the change to output request based on the accumulation of error present
            double d = kd * (error - lastError) / elapsedTime;//derivative component, bases the change to output request based on the rate of change of error present
            double pid = p + i + d;
            zero.setPower(-pid);
            one.setPower(+pid);
            two.setPower(-pid);
            three.setPower(+pid);
            telemetry.addData("Heading", getYaw());
            telemetry.update();
            sleep(1);
            accumulatedError += error;
            lastError = error;
            elapsedTime = System.currentTimeMillis() - currentTime;
        }
    }
    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
    }
    public double getYaw() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public double feedback(double x, double target) {
        double compressionFactor = 160; //Must be > 0 and positive. Larger compression factor means gentler x velocity changes

        return x <= target ? Math.max(((-Math.pow(target,3)*Math.sqrt(1-(Math.pow(x,2)/Math.pow(target,2))))/Math.pow(x,2))/compressionFactor,-Math.sqrt(3)/2.0) : Math.min(((Math.pow(xMax-target,2)*Math.sqrt(1-Math.pow((x-xMax)/(xMax-target),2)))/Math.abs(x-xMax))/compressionFactor,Math.sqrt(3)/2.0);
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        retina = Retina.create(new Size(320,180));
        retina.setup();
        retina.clearBuffers();
    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat input = inputFrame.rgba().clone();
        inputFrame.rgba().release();
        inputFrame.gray().release();

        Imgproc.resize(input,input,new Size(input.size().width/4,input.size().height/4)); //Reduces image size for speed

        input.convertTo(input,CvType.CV_32F);
        retina.applyFastToneMapping(input,input);
        input.convertTo(input,CvType.CV_8U);

        //Defines all Mats that will be used in the program
        Mat lab = new Mat();
        Mat labThreshBinary = new Mat();
        Mat labThreshOtsu = new Mat();
        Mat labThresh = new Mat();
        Mat hsv = new Mat();
        Mat hChan = new Mat();
        Mat bChan = new Mat();

        //Converts input from RGB color format to Lab color format, then extracts the b channel
        //Lab is based on the opponent color model, and the b channel represents the blue-yellow axis, so it will be useful in finding yellow colors
        Imgproc.cvtColor(input,lab,Imgproc.COLOR_RGB2Lab);
        Core.extractChannel(lab,bChan,2);

        //Removes used images from memory to avoid overflow crashes
        lab.release();

        /*Thresholds the b channel in two different ways to get a binary filter (correct or not correct)
        for all detected yellow pixels
        The binary threshold selects all pixels with a b value above 145
        The Otsu threshold does the same thing as the binary threshold, but tries to dynamically
        select the threshold value (the value above which a pixel is considered yellow) to divide the
        image by contrast. The binary threshold is very inclusive, for reasons that will become clear later*/
        Imgproc.threshold(bChan,labThreshBinary,145,255,Imgproc.THRESH_BINARY);
        Imgproc.threshold(bChan,labThreshOtsu,0,255,Imgproc.THRESH_OTSU);

        /*Otsu threshold will usually do a good job of segmenting the cubes from the rest of the
        image (as they contrast heavily with the background), but does not function well when there
        are no cubes in the image, as the optimal contrast threshold will not necessarily be filtering
        for yellow. The binary threshold, however, is not affected by the absence of the cubes, and so
        by performing a bitwise and of the images, we keep only the area where both thresholds agree,
        which accounts for times when the cube is not in the image while keeping the otsu threshold's power*/
        Core.bitwise_and(labThreshBinary,labThreshOtsu,labThresh);

        //Removes used images from memory to avoid overflow crashes
        bChan.release();
        labThreshBinary.release();
        labThreshOtsu.release();

        //Converts input from RGB color format to HSV color format, then extracts the h channel
        //HSV stands for hue, saturation, value. We are only interested in the h channel, which stores color information
        //Because of its division of color into a separate channel, HSV format is resistant to lighting changes and so is good for color filtering
        Imgproc.cvtColor(input,hsv,Imgproc.COLOR_RGB2HSV_FULL);
        Core.extractChannel(hsv,hChan,0);

        //Masks image so that the only h regions detected are those that were also detected by the Lab otsu and binary thresholds
        Mat masked = new Mat();
        Core.bitwise_and(hChan,labThresh,masked);

        //Removes used images from memory to avoid overflow crashes
        hsv.release();
        hChan.release();

        /*Computes the distance transform of the Lab image threshold and then does a binary threshold of that
        The distance transform sorts pixels by their distance from the nearest black pixel. Larger distance means a higher value
        This is done here in order to reduce noise, which will have a small distance from black pixels*/
        Mat distanceTransform = new Mat();
        Mat thresholded = new Mat();
        Imgproc.distanceTransform(labThresh,distanceTransform,Imgproc.DIST_L2,3);
        distanceTransform.convertTo(distanceTransform,CvType.CV_8UC1);
        Imgproc.threshold(distanceTransform,thresholded,0,255,Imgproc.THRESH_OTSU);

        //Removes used images from memory to avoid overflow crashes
        distanceTransform.release();

        //Performs a gaussian blur on the threshold to help eliminate remaining high frequency noise
        Imgproc.GaussianBlur(thresholded,thresholded,new Size(3,3),0);

        //Finds all detected blobs in the thresholded distance transform and finds their centers
        List<MatOfPoint> centerShapes = new ArrayList<>();
        Imgproc.findContours(thresholded,centerShapes,new Mat(),Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_SIMPLE);
        thresholded.release();
        List<Point> centers = new ArrayList<>();
        for(MatOfPoint shape : centerShapes) {
            Moments moments = Imgproc.moments(shape);
            centers.add(new Point(moments.m10/moments.m00,moments.m01/moments.m00));
            shape.release();
        }

        //Removes used images from memory to avoid overflow crashes
        labThresh.release();

        //Calculates the median value of the image
        double med = getMedian(masked);

        //Dynamically calculates the best parameters for the Canny edge detector to find the edges of all of the detected shapes
        //Edges are represented as a binary image, with "on" pixels along the edge and "off" pixels everywhere else
        Mat edges = new Mat();
        double sigma = 0.33;
        Imgproc.Canny(masked,edges,(int) Math.round(Math.max(0,(1-sigma)*med)),(int) Math.round(Math.min(255,1+sigma)*med));

        //Enhances edge information
        Imgproc.dilate(edges,edges,Imgproc.getStructuringElement(Imgproc.MORPH_CROSS,new Size(5,5)),new Point(),2);

        //Turns edges into a list of shapes
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges,contours,new Mat(),Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE);

        //Removes used images from memory to avoid overflow crashes
        edges.release();
        double a = 0;
        double b = 0;
        double maxSize = 0;
        //Loops through the list of shapes (contours) and finds the ones most likely to be a cube
        for (int i = 0; i < contours.size(); i++) {
            //Approximates the shape to smooth out excess edges
            MatOfPoint2f approx = new MatOfPoint2f();
            double peri = Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), true);
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), approx, 0.1 * peri, true); //0.1 is a detail factor, higher factor = lower detail, lower factor = higher detail
            MatOfPoint approxMop = new MatOfPoint(approx.toArray());

            //Does a simple size check to eliminate extremely small contours
            if (Imgproc.contourArea(approxMop) > 10) {
                //Checks if one of the distance transform centers is contained within the shape
                if (containsPoint(approx,centers)) {
                    //Calculates a convex hull of the shape, covering up any dents
                    MatOfPoint convex = hull(approxMop);
                    //Calculates a rectangle that lies completely inside the shape
                    Rect bbox = calcBox(convex);

                    //Size check to see if the box could be calculated
                    if (bbox.x >= 0 && bbox.y >= 0 && bbox.x + bbox.width <= masked.cols() && bbox.y + bbox.height <= masked.rows()) {
                        //Selects the region of interest (roi) determined from the calcBox function from the masked h channel image
                        Mat roi = masked.submat(bbox);
                        //Calculates the standard deviation and mean of the selected region. In this case it will calculate the average color and the color standard deviation
                        double[] stdMean = calcStdDevMean(roi);

                        //Does a test for average color and standard deviation (average color between 10 and 40, exclusive, and standard deviation less than 24)
                        if (stdMean[1] > 10 && stdMean[1] < 40 && stdMean[0] < 24) {
                            //Calculate the overall bounding rectangle around the shape
                            Rect bboxLarge = Imgproc.boundingRect(approxMop);

                            Imgproc.putText(input, Double.toString(Math.floor(100*(1.0 * bboxLarge.width) / (1.0 * bboxLarge.height))/100.0), new Point(bbox.x, bbox.y), Core.FONT_HERSHEY_COMPLEX, 1, new Scalar(255, 0, 0), 3);

                            //Checks the size of the bounding box against what it can be based on a model of a rotating cube. Tolerance is added to account for noise
                            double tolerance = 0.25; //must be positive
                            if (((1.0 * bboxLarge.width) / (1.0 * bboxLarge.height)) > Math.sqrt(2.0 / 3.0) * (1 - tolerance) && ((1.0 * bboxLarge.width) / (1.0 * bboxLarge.height)) < Math.sqrt(3.0 / 2.0) * (1 + tolerance)) {
                                //Checks if shape has 4 or 6 corners, which will be true for any cube-shaped object
                                if (approx.toList().size() == 4 || approx.toList().size() == 6) {
                                    //Draws shape to screen
                                    Imgproc.drawContours(input, contours, i, new Scalar(0, 255, 0), 9);
                                    if(Imgproc.contourArea(approx) > maxSize) {
                                        maxSize = Imgproc.contourArea(approx);
                                        Moments m = Imgproc.moments(approx);
                                        a = m.m10/m.m00;
                                        b = m.m01/m.m00;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            //Removes used images from memory to avoid overflow crashes
            contours.get(i).release();
            approx.release();
            approxMop.release();
        }
        //Removes used images from memory to avoid overflow crashes
        masked.release();

        //Empties the cosmic garbage can
        System.gc();

        Imgproc.resize(input,input,new Size(1280,720));

        prevCenter = blockCenter.clone();
        blockCenter = new Point(a,b);

        return input;
    }
    //Check if polygon contains any point in a list of points. True if polygon contains any point in the list, false otherwise
    private boolean containsPoint(MatOfPoint2f polygon, List<Point> points) {
        boolean inPolygon = false;
        for(Point p : points) {
            if(Imgproc.pointPolygonTest(polygon, p, false) >= 0) { //returns value > 0 if in polygon, 0 if on the edge of the polygon, and < 0 if outside the polygon
                inPolygon = true;
            }
        }
        return inPolygon;
    }

    //Calculates rectangle completely inside the shape
    private Rect calcBox(MatOfPoint c) {
        //Calculates center of the shape wrong
        Moments m = Imgproc.moments(c);
        Point center = new Point(m.m10/m.m00,m.m01/m.m00);

        //Gets list of the shape's corners
        List<Point> corners = c.toList();

        //Finds the smallest distance from the center to one of the corners
        double minDst = Integer.MAX_VALUE;
        double dst;
        for(Point p : corners) {
            dst = dist(center,p);
            minDst = dst < minDst ? dist(center,p) : minDst; //min distance equals the distance between the center and the current corner if the distance is less than the previous min distance
        }

        //Calculate and return rectangle coordinates assuming the rectangle is inside a circle of radius minDst
        return new Rect(new Point(Math.round(center.x-minDst/Math.sqrt(2)),(int) Math.round(center.y-minDst/Math.sqrt(2))),new Point(Math.round(center.x+minDst/Math.sqrt(2)),(int) Math.round(center.y+minDst/Math.sqrt(2))));
    }

    //Calculates standard deviation and mean of an image. Output is a constant list of doubles with the following format: {standard deviation, mean}
    private double[] calcStdDevMean(Mat input) {
        assert input.channels() == 1: "input must only have 1 channel"; //Makes sure image is only 1 channel (ex: black and white)

        //Calculates image mean and standard deviation
        MatOfDouble std = new MatOfDouble();
        MatOfDouble mean = new MatOfDouble();
        Core.meanStdDev(input,mean,std);
        double[] output = new double[] {std.get(0,0)[0],mean.get(0,0)[0]};

        //Removes used images from memory to avoid overflow crashes
        std.release();
        mean.release();

        //returns output data with the following format: {standard deviation, mean}
        return output;
    }

    //Gets the median value of the image
    private double getMedian(Mat input) {
        //Turns image into a single row of pixels
        Mat rowMat = input.reshape(0,1);

        //Sort pixel values from least to greatest
        Mat sorted = new Mat();
        Core.sort(rowMat,sorted,Core.SORT_ASCENDING);

        //Calculates median of the image. Median is the middle value of the row of sorted pixels. If there are two middle pixels, the median is their average.
        double median = sorted.size().width % 2 == 1 ? sorted.get(0,(int) Math.floor(sorted.size().width/2))[0] : (sorted.get(0,(int) (sorted.size().width/2)-1)[0]+sorted.get(0,(int) sorted.size().width/2)[0])/2;

        //Removes used images from memory to avoid overflow crashes
        rowMat.release();
        sorted.release();

        return median;
    }

    //Calculates distance between two points
    private double dist(Point a, Point b) {
        return(Math.sqrt(Math.pow(a.x-b.x,2)+Math.pow(a.y-b.y,2))); //distance formula
    }

    //Calculates convex hull of a shape
    private MatOfPoint hull(MatOfPoint mopIn) { //mop = MatOfPoint

        //Calculates indexes of convex points on the shape
        MatOfInt hull = new MatOfInt();
        Imgproc.convexHull(mopIn, hull, false);

        //Creates output shape to store convex points
        MatOfPoint mopOut = new MatOfPoint();
        mopOut.create((int)hull.size().height,1,CvType.CV_32SC2);

        //Selects all convex points (at the calculated hull indices) and adds them to the output shape
        for(int i = 0; i < hull.size().height ; i++)
        {
            int index = (int)hull.get(i, 0)[0];
            double[] point = new double[] {
                    mopIn.get(index, 0)[0], mopIn.get(index, 0)[1]
            };
            mopOut.put(i, 0, point);
        }

        //Removes used images from memory to avoid overflow crashes
        hull.release();

        return mopOut;
    }
    public void startOpenCV(CameraBridgeViewBase.CvCameraViewListener2 cameraViewListener) {
        FtcRobotControllerActivity.turnOnCameraView.obtainMessage(1, cameraViewListener).sendToTarget();
    }

    public void stopOpenCV() {
        FtcRobotControllerActivity.turnOffCameraView.obtainMessage().sendToTarget();
    }
}