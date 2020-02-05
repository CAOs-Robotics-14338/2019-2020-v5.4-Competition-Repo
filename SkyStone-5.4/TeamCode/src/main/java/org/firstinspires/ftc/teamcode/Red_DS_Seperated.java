package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name= "Red DS Seperated", group="Red")
public class Red_DS_Seperated extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor, IntakeLeftMotor, IntakeRightMotor;
    private Servo left_hook, right_hook, IntakePulley;
    HolonomicDrive holonomicDrive;
    Intake_Systems intake_systems;
    BotServos botServos;
    gyro Gyro;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;


    double lStored = 0;
    double rStored = 1;
    double lActive = 0.6;
    double rActive = 0.4;
    double servoTime = 0.6;
    double time = 2;
    double intake_time = 0.50;
    double wallToSS1 = 1.45; //1.55
    double wallToSS2 = 1.45; //1.35
    double wallToSS3 = 1.45;
    double SS1ToFoundation = 1.45;
    double SS2ToFoundation = 1.85;
    double SS3ToFoundation = 2.05; //1.65
    double pos1FND2SS2 = 2.5;
    double pos2FND2SS2 = 2.5; //2.4
    double pos3FND2SS2 = 2.7; //2.9      2.5
    double P1SS2ToFoundation = 3.05; // 3.15      2.55
    double P2SS2ToFoundation = 2.85; //2.85
    double P3SS2ToFoundation = 3.05; //2.45    2.95
    double Foundation2Skybride = 0.5;

    private double r_time = 3.5;
    private double numTime = 0.5;
    private double postime = 0;
    private double pos2time = 0;
    private double movetime = 0;
    private double newtime = 0;
    private double timetime = 0;
    private double newtime2 = 0;
    int pos;
    boolean skyFound = false;
    boolean sky2Found = false;
    boolean done = false;

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;
    boolean posfound = false;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = -2.5f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 1.7f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {3.5f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2.0f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        FrontRightMotor = hardwareMap.get(DcMotor.class, "front_right_drive");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        BackRightMotor = hardwareMap.get(DcMotor.class, "back_right_drive");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
        IntakeLeftMotor = hardwareMap.get(DcMotor.class, "left_intake");
        IntakeRightMotor = hardwareMap.get(DcMotor.class, "right_intake");
        //ScissorLiftMotor =  hardwareMap.get(DcMotor.class, "scissor");
        IntakePulley = hardwareMap.servo.get("intake_pulley");
        left_hook = hardwareMap.servo.get("left_hook");
        right_hook = hardwareMap.servo.get("right_hook");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled      = false;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        holonomicDrive = new HolonomicDrive(FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor);
        //scissorLift = new ScissorLift(ScissorLiftMotor);
        intake_systems = new Intake_Systems(IntakeRightMotor, IntakeLeftMotor, IntakePulley);
        Gyro = new gyro(FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor, imu);
        botServos = new BotServos(left_hook, right_hook);


        // Setting servos to the retracted position
        left_hook .setPosition(lStored);
        right_hook.setPosition(rStored);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam Red"), cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_RIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.update();
            sleep(100);
            if(valMid == 0 && !posfound){pos = 2; posfound = true;}
            else if(valRight == 0 && !posfound){pos = 1; posfound = true;}
            else{pos = 3; posfound = true;}


            if(pos == 1 && !skyFound){
                skyFound = true;

                // Driving from the wall to the first skystone @ position 1
                runtime.reset();
                holonomicDrive.autoDrive(0,0.9);
                while (opModeIsActive() && runtime.seconds() < wallToSS1){
                    telemetry.addLine("Driving to 1st Skystone at position 1");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();

                // Rotating to be @ a 20 degree angle to the skystone
                Gyro.rotate(25,0.4);
                sleep(100);

                // Starting our intake wheel to collect the skystone
                intake_systems.intake(true, false);
                runtime.reset();
                holonomicDrive.autoDrive(0,0.9);
                while (opModeIsActive() && runtime.seconds() < intake_time){
                    telemetry.addLine("Collecting the Skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                sleep(250);

                // Reversing from collecting so we can drive under the alliance bridge
                runtime.reset();
                holonomicDrive.autoDrive(180,0.8);
                while (opModeIsActive() && runtime.seconds() < intake_time+0.4){
                    telemetry.addLine("Reversing to drive under the bridge");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();

                // Stopping collection wheels and rotating to point towards the building zone
                intake_systems.intake(false, false);
                Gyro.rotate(-107, 0.5);
                sleep(200);

                // Driving towards the building site with skystone in intake
                runtime.reset();
                holonomicDrive.autoDrive(0,0.80);
                while (opModeIsActive() && runtime.seconds() < SS1ToFoundation){
                    telemetry.addLine("Driving to the building site");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                intake_systems.intake(false, true);
                sleep(200);

                // Driving to the second skystone
                runtime.reset();
                holonomicDrive.autoDrive(160, 0.90); //200
                while (opModeIsActive() && runtime.seconds() < pos1FND2SS2){
                    telemetry.addLine("Returning to second skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();

                // Intaking the second skystone
                Gyro.rotate(105,0.5); //115
                sleep(150);
                intake_systems.intake(true,false);
                runtime.reset();
                holonomicDrive.autoDrive(0, 0.90);
                while (opModeIsActive() && runtime.seconds() < intake_time+1){
                    telemetry.addLine("Collecting the second skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();



                // Reversing with the second skystone so we can drive under the sky bridge
                runtime.reset();
                holonomicDrive.autoDrive(180,0.90);
                while (opModeIsActive() && runtime.seconds() < intake_time+0.55){
                    // Adding telemetry of the time elapsed
                    telemetry.addLine("Reversing with the skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                intake_systems.intake(false,false);

                // Rotating to face the building site
                Gyro.rotate(-95,0.5);
                sleep(150);

                // Driving to the building site with the second skystone
                runtime.reset();
                holonomicDrive.autoDrive(0,0.90);
                while (opModeIsActive() && runtime.seconds() < P1SS2ToFoundation){
                    // Adding telemetry of the time elapsed
                    telemetry.addLine("Driving to foundation with second skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                intake_systems.intake(false,true);


                // Reversing under the skybridge
                runtime.reset();
                holonomicDrive.autoDrive(180,0.90);
                while (opModeIsActive() && runtime.seconds() < Foundation2Skybride){
                    // Adding telemetry of the time elapsed
                    telemetry.addLine("Parking under the skybridge");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                intake_systems.intake(false,false);

                done = true;
                stop();


            }
            else if(pos == 2 && !skyFound){
                skyFound = true;

                // Driving from the wall to the first skystone @ position 1
                runtime.reset();
                holonomicDrive.autoDrive(330,0.90);
                while (opModeIsActive() && runtime.seconds() < wallToSS2){
                    telemetry.addLine("Driving to 1st Skystone at position 2");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();

                // Rotating to be @ a 15 degree angle to the skystone
                Gyro.rotate(15,0.5); //10
                sleep(100);

                // Starting our intake wheel to collect the skystone
                intake_systems.intake(true, false);
                runtime.reset();
                holonomicDrive.autoDrive(0,0.8);
                while (opModeIsActive() && runtime.seconds() < intake_time+0.2){
                    telemetry.addLine("Collecting the Skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                sleep(250);

                // Reversing from collecting so we can drive under the alliance bridge
                runtime.reset();
                holonomicDrive.autoDrive(180,0.8);
                while (opModeIsActive() && runtime.seconds() < intake_time+0.35){
                    telemetry.addLine("Reversing to drive under the bridge");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();

                // Stopping collection wheels and rotating to point towards the building zone
                intake_systems.intake(false, false);
                Gyro.rotate(-100, 0.5); //-100  -105
                sleep(150);

                // Driving towards the building site with skystone in intake
                runtime.reset();
                holonomicDrive.autoDrive(0,0.90);
                while (opModeIsActive() && runtime.seconds() < SS2ToFoundation){
                    telemetry.addLine("Driving to the building site");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                intake_systems.intake(false, true);


                // Moving to the second skystone
                runtime.reset();
                holonomicDrive.autoDrive(160, 0.95);
                while (opModeIsActive() && runtime.seconds() < pos2FND2SS2){
                    telemetry.addLine("Returning to second skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();

                // Intaking the second skystone
                Gyro.rotate(112,0.5); // 107             105 115
                sleep(150);
                intake_systems.intake(true,false);
                runtime.reset();
                holonomicDrive.autoDrive(0, 0.85);
                while (opModeIsActive() && runtime.seconds() < intake_time+0.5+0.4){
                    telemetry.addLine("Collecting the second skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();

                // Reversing with the second skystone so we can drive under the sky bridge
                runtime.reset();
                holonomicDrive.autoDrive(180,0.90);
                while (opModeIsActive() && runtime.seconds() < intake_time+0.3 +0.4){
                    // Adding telemetry of the time elapsed
                    telemetry.addLine("Reversing with the skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                intake_systems.intake(false,false);

                // Rotating to face the building site
                Gyro.rotate(-95,0.5); //-105
                sleep(150);

                // Driving to the building site with the second skystone
                runtime.reset();
                holonomicDrive.autoDrive(0,0.90);
                while (opModeIsActive() && runtime.seconds() < P2SS2ToFoundation){
                    // Adding telemetry of the time elapsed
                    telemetry.addLine("Driving to foundation with second skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                intake_systems.intake(false,true);

                // Reversing under the skybridge
                runtime.reset();
                holonomicDrive.autoDrive(180,0.90);
                while (opModeIsActive() && runtime.seconds() < Foundation2Skybride){
                    // Adding telemetry of the time elapsed
                    telemetry.addLine("Parking under the skybridge");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                done = true;
                stop();


            }
            else if(pos == 3 && !skyFound) {
                skyFound = true;

                // Driving from the wall to the first skystone @ position 1
                runtime.reset();
                holonomicDrive.autoDrive(320,0.90);
                while (opModeIsActive() && runtime.seconds() < wallToSS3){
                    telemetry.addLine("Driving to 1st Skystone at position 3");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();

                // Rotating to be @ a 15 degree angle to the skystone
                Gyro.rotate(12,0.5);
                sleep(100);

                // Starting our intake wheel to collect the skystone
                intake_systems.intake(true, false);
                runtime.reset();
                holonomicDrive.autoDrive(0,0.8);
                while (opModeIsActive() && runtime.seconds() < intake_time+0.7){
                    telemetry.addLine("Collecting the Skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                sleep(250);

                // Reversing from collecting so we can drive under the alliance bridge
                runtime.reset();
                holonomicDrive.autoDrive(180,0.8);
                while (opModeIsActive() && runtime.seconds() < intake_time+0.4){
                    telemetry.addLine("Reversing to drive under the bridge");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();

                // Stopping collection wheels and rotating to point towards the building zone
                intake_systems.intake(false, false);
                Gyro.rotate(-100, 0.5); // -105
                sleep(150);

                // Driving towards the building site with skystone in intake
                runtime.reset();
                holonomicDrive.autoDrive(0,0.90);
                while (opModeIsActive() && runtime.seconds() < SS3ToFoundation+0.2){
                    telemetry.addLine("Driving to the building site");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();

                intake_systems.intake(false, true);

                // Moving to the second skystone
                runtime.reset();
                holonomicDrive.autoDrive(160, 0.95);
                while (opModeIsActive() && runtime.seconds() < pos3FND2SS2+0.2){
                    telemetry.addLine("Returning to second skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();

                // Intaking the second skystone
                Gyro.rotate(115,0.5);
                sleep(150);
                intake_systems.intake(true,false);
                runtime.reset();
                holonomicDrive.autoDrive(0, 0.85);
                while (opModeIsActive() && runtime.seconds() < intake_time+1.3){
                    telemetry.addLine("Collecting the second skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();

                // Reversing with the second skystone so we can drive under the sky bridge
                runtime.reset();
                holonomicDrive.autoDrive(180,0.90);
                while (opModeIsActive() && runtime.seconds() < intake_time+0.45){
                    // Adding telemetry of the time elapsed
                    telemetry.addLine("Reversing with the skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                intake_systems.intake(false,false);

                // Rotating to face the building site
                Gyro.rotate(-100,0.5); //-120  -115    -110   ::: When the wall is not held in place with a foot or structual rigidity, the robot will turn ~~15 degrees sliding on the wall.
                // This value is -110 when the wall is held in place/doesn't move &&&&&& -100 when the wall isn't held in place/moves
                sleep(150);

                // Driving to the building site with the second skystone
                runtime.reset();
                holonomicDrive.autoDrive(0,0.90);
                while (opModeIsActive() && runtime.seconds() < P3SS2ToFoundation){
                    // Adding telemetry of the time elapsed
                    telemetry.addLine("Driving to foundation with second skystone");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                intake_systems.intake(false,true);


                // Reversing under the skybridge
                runtime.reset();
                holonomicDrive.autoDrive(180,0.90);
                while (opModeIsActive() && runtime.seconds() < Foundation2Skybride+0.2){
                    // Adding telemetry of the time elapsed
                    telemetry.addLine("Parking under the skybridge");
                    telemetry.update();
                }
                holonomicDrive.stopMoving();
                intake_systems.intake(false,false);

                done = true;
                stop();





            }
            else{
                telemetry.addData("Sorry!", "I don't see anything!");
                telemetry.update();
            }

        }

    }




    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}
