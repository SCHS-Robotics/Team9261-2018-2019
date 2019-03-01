package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
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

@TeleOp(name="Gold Miner", group="OpMode")
public class GoldMine2 extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2{

    private Retina retina;
    private NonMaxSuppressor nonMaxSuppressor;

    int xMax = 320;
    int yMax = 180;

    List<TrackedObject> tracked = new ArrayList<>();
    List<TrackedObject> newTracks = new ArrayList<>();
    List<Point> used = new ArrayList<>();
    List<Rect> rects = new ArrayList<>();
    List<Rect> prevRects = new ArrayList<>();

    Point blockCenter = new Point(-1,-1);

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
        retina = Retina.create(new Size(320,180));
        retina.setup();
        retina.clearBuffers();

        nonMaxSuppressor = new NonMaxSuppressor(0.3);
    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        Mat input = inputFrame.rgba().clone();
        inputFrame.rgba().release();
        inputFrame.gray().release();

        Imgproc.resize(input, input, new Size(320, (int) Math.round((320/input.size().width)*input.size().height))); //Reduces image size for speed

        retina.applyFastToneMapping(input,input);

        Mat yuvThresh = new Mat();
        Mat labThresh = new Mat();
        Mat intensityMap = new Mat();

        YUVProcess yuvProcess = new YUVProcess(input,yuvThresh);
        HSV_LabProcess hsv_labProcess = new HSV_LabProcess(input,intensityMap,labThresh);
        yuvProcess.run();
        hsv_labProcess.run();

        while(yuvProcess.running || hsv_labProcess.running);

        Core.bitwise_and(labThresh,yuvThresh,labThresh);

        yuvThresh.release();

        Imgproc.morphologyEx(labThresh,labThresh,Imgproc.MORPH_CLOSE,Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(5,5)));

        /*Computes the distance transform of the Lab image threshold and then does a binary threshold of that
        The distance transform sorts pixels by their distance from the nearest black pixel. Larger distance means a higher value
        This is done here in order to reduce noise, which will have a small distance from black pixels*/
        Mat distanceTransform = new Mat();
        Mat thresholded = new Mat();
        Imgproc.distanceTransform(labThresh,distanceTransform,Imgproc.DIST_L2,3);
        distanceTransform.convertTo(distanceTransform,CvType.CV_8UC1);


        Mat msk = new Mat();
        Imgproc.threshold(distanceTransform,msk,0,255,Imgproc.THRESH_BINARY);

        double stdm[] = calcStdDevMean(distanceTransform,msk);

        Imgproc.threshold(distanceTransform,thresholded,stdm[1]/stdm[0],255,Imgproc.THRESH_BINARY);

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
            centers.add(getCenter(shape));
            shape.release();
        }

        //Removes used images from memory to avoid overflow crashes
        //labThresh.release();

        //Dynamically calculates the best parameters for the Canny edge detector to find the edges of all of the detected shapes
        //Edges are represented as a binary image, with "on" pixels along the edge and "off" pixels everywhere else
        Mat edges = new Mat();
        Imgproc.Canny(labThresh,edges,0,255);

        //Enhances edge information
        Imgproc.dilate(edges,edges,Imgproc.getStructuringElement(Imgproc.MORPH_CROSS,new Size(2,2)),new Point(),1);

        //Turns edges into a list of shapes
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges,contours,new Mat(),Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE);

        //Removes used images from memory to avoid overflow crashes
        edges.release();

        Core.MinMaxLocResult minMaxLocResult = Core.minMaxLoc(intensityMap);
        double max = minMaxLocResult.maxVal;

        List<Rect> bboxes = new ArrayList<>();

        List<Double> usedx = new ArrayList<>();
        List<Double> usedy = new ArrayList<>();
        //Loops through the list of shapes (contours) and finds the ones most likely to be a cube
        for (int i = 0; i < contours.size(); i++) {
            //Approximates the shape to smooth out excess edges
            MatOfPoint2f approx = new MatOfPoint2f();
            double peri = Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), true);
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), approx, 0.05 * peri, true); //0.1 is a detail factor, higher factor = lower detail, lower factor = higher detail
            MatOfPoint approxMop = new MatOfPoint(approx.toArray());
            //Calculates a convex hull of the shape, covering up any dents
            MatOfPoint convex = hull(approxMop);
            //Does a simple size check to eliminate extremely small contours
            if (Imgproc.contourArea(approxMop) > 100) {

                //Checks if one of the distance transform centers is contained within the shape

                //Imgproc.putText(input,"1",new Point(box.x, box.y), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(0, 0, 0), 3);
                Point center = getCenter(approxMop);
                if (containsPoint(approx,centers) && !(usedx.contains(center.x) && usedy.contains(center.y))) {
                    usedx.add(center.x);
                    usedy.add(center.y);
                    Rect box = Imgproc.boundingRect(convex);
                    Mat roi = intensityMap.submat(box);
                    Core.MinMaxLocResult res = Core.minMaxLoc(roi);
                    if(res.maxVal >= 0.75*max) {
                        Imgproc.drawContours(input, contours, i, new Scalar(255,0, 0), 1);
                        if(convex.toList().size() == 4 || convex.toList().size() == 5 || convex.toList().size() == 6) {
                            Imgproc.drawContours(input, contours, i, new Scalar(0,0, 255), 1);
                            Rect bbox = Imgproc.boundingRect(convex);
                            System.out.println((1.0*bbox.width)/(1.0*bbox.height));
                            if((1.0*bbox.width)/(1.0*bbox.height) >= Math.sqrt(2)/2.0 && (1.0*bbox.width)/(1.0*bbox.height) <= Math.sqrt(2)) {
                                Imgproc.drawContours(input, contours, i, new Scalar(0,255, 0), 1);
                                bboxes.add(bbox);
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

        List<Rect> goodBoxes = nonMaxSuppressor.suppressNonMax(bboxes);

        for(Rect box: goodBoxes) {
            Imgproc.rectangle(input,new Point(box.x,box.y),new Point(box.x+box.width,box.y+box.height),new Scalar(0,255,0),1);
            rects.add(box);
        }
        for (Rect r : rects) {
            Point center = new Point(r.x + r.width / 2.0, r.y + r.height / 2.0);
            boolean foundMatch = false;
            for (TrackedObject t1 : tracked) {
                if(t1.r.contains(center)) {
                    t1.r = r;
                    t1.kt.update(center,true);
                    newTracks.add(t1);
                    foundMatch = true;
                    break;
                }
            }
            if (!foundMatch) {
                TrackedObject t2 = new TrackedObject(r, new KalmanTracker(center));
                t2.kt.update(center,true);
                newTracks.add(t2);
            }
        }

        rects.clear();

        for(TrackedObject t : tracked) {
            if(!t.kt.wasUpdated && t.kt.lastUpdateTimer < 1000 && t.kt.trustworthyness > 0.5) {
                t.kt.update(new Point(),false);
                Point center = new Point(t.r.x + t.r.width / 2.0, t.r.y + t.r.height / 2.0);
                t.r = new Rect(Math.max(0, (int) Math.round(t.r.x + (t.kt.lastResult.x - center.x))), Math.max(0, (int) Math.round(t.r.y + (t.kt.lastResult.y - center.y))), t.r.width, t.r.height);
                newTracks.add(t);
            }
        }

        used.clear();

        tracked.clear();
        tracked.addAll(newTracks);
        newTracks.clear();

        for(TrackedObject t : tracked) {


            Point prediction = t.kt.getPrediction();
            Imgproc.circle(input,prediction,5,new Scalar(0,0,255),-1);

            Imgproc.putText(input,Double.toString(Math.round(100*t.kt.trustworthyness)/100.0),prediction,Core.FONT_HERSHEY_PLAIN,1, new Scalar(255,0,0));

            //System.out.println(t.kt.trustworthyness);
            //Imgproc.circle(mat,prediction,5,new Scalar(0,0,255),-1);
        }
        prevRects.clear();
        prevRects.addAll(rects);

        blockCenter = getMaxTrust(tracked);

        //Empties the cosmic garbage can
        System.gc();


        Mat returnImage = input;
        Imgproc.resize(returnImage,returnImage,new Size(1280,720));
        return returnImage;
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

    private Point getCenter(MatOfPoint c) {
        Moments m = Imgproc.moments(c);
        return new Point(m.m10/m.m00,m.m01/m.m00);
    }
    //Calculates rectangle completely inside the shape
    private Rect calcBox(MatOfPoint c) {
        //Calculates center of the shape
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

    private double getMedianNonZero(Mat input) {
        //Turns image into a single row of pixels
        Mat rowMat = input.reshape(0,1);

        //Sort pixel values from least to greatest
        Mat sorted = new Mat();
        Core.sort(rowMat,sorted,Core.SORT_ASCENDING);

        double sum = 0;
        int idx = 0;
        int loops = 0;
        while(sum == 0 && loops < sorted.cols() && opModeIsActive()) {
            sum+=sorted.get(0,loops)[0];
            idx+=sorted.get(0,loops)[0] > 0 ? 1 : 0;
            loops++;
        }

        //Calculates median of the image. Median is the middle value of the row of sorted pixels. If there are two middle pixels, the median is their average.
        double median = (sum != 0 ) ? ((sorted.size().width-idx) % 2 == 1 ? sorted.get(0,(int) Math.floor(idx+((sorted.size().width-idx)/2)))[0] : (sorted.get(0,(int) (idx+(sorted.size().width-idx)/2)-1)[0]+sorted.get(0,(int) (idx+(sorted.size().width-idx)/2))[0])/2) : 0;

        //Removes used images from memory to avoid overflow crashes
        rowMat.release();
        sorted.release();

        return median;
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
    private double[] calcStdDevMean(Mat input, Mat mask) {
        assert input.channels() == 1: "input must only have 1 channel"; //Makes sure image is only 1 channel (ex: black and white)

        //Calculates image mean and standard deviation
        MatOfDouble std = new MatOfDouble();
        MatOfDouble mean = new MatOfDouble();
        Core.meanStdDev(input,mean,std,mask);
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

    private Point getMaxTrust(List<TrackedObject> tl) {
        double maxTrust = 0;
        Point trusted = new Point(-1,-1);
        Point trustedBackup = new Point(-1,-1);
        for(TrackedObject t : tl) {
            if(t.kt.trustworthyness >= maxTrust && !t.kt.isFirstUpdate) {
                maxTrust = t.kt.trustworthyness;
                trusted = t.kt.lastResult;
            }
            else if(t.kt.isFirstUpdate) {
                trustedBackup = t.kt.lastResult;
            }
        }

        return trusted.x >=0 ? trusted : trustedBackup;
    }

    public void startOpenCV(CameraBridgeViewBase.CvCameraViewListener2 cameraViewListener) {
        FtcRobotControllerActivity.turnOnCameraView.obtainMessage(1, cameraViewListener).sendToTarget();
    }

    public void stopOpenCV() {
        FtcRobotControllerActivity.turnOffCameraView.obtainMessage().sendToTarget();
    }
}