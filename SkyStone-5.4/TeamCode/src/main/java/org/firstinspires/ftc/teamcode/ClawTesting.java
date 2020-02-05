package org.firstinspires.ftc.teamcode;

//Importing required classes
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Declaring autonomous named Servo_Autonomous with the ground test
@Autonomous(name="Claw testing", group="Blue")
@Disabled
// Creating class named servo autonomous that uses linear op mode
public class ClawTesting extends LinearOpMode {

    double lStored = 0;
    double rStored = 1;
    double lActive = 0.6;
    double rActive = 0.4;


    // Declaring servos attached/required for this autonomous class
    Servo left_hook, right_hook;
    //Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();

    // Declaring the DC motors for our holonomic drive base
    private DcMotor FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor;
    private Servo IntakePulley, claw, wrist;
    private CRServo expansion;
    HolonomicDrive holonomicDrive;
    ArmCollection armCollection;

    // Starting OPMode
    @Override
    public void runOpMode() {

        // Classifying our Dc motors with their names on the expansion hub
        FrontRightMotor  = hardwareMap.get(DcMotor.class, "front_right_drive");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        BackRightMotor  = hardwareMap.get(DcMotor.class, "back_right_drive");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
        IntakePulley = hardwareMap.servo.get("intake_pulley");
        left_hook = hardwareMap.servo.get("left_hook");
        right_hook = hardwareMap.servo.get("right_hook");

        claw = hardwareMap.servo.get("claw");
        wrist = hardwareMap.servo.get("wrist");
        expansion = hardwareMap.get(CRServo.class,"expansion");

        // Setting our holonomic drive to use our 2 front and 2 back motors
        holonomicDrive = new HolonomicDrive(FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor);
        armCollection = new ArmCollection(claw, wrist, expansion, IntakePulley);


        // Classifying our servos with their names on the expansion hub
        left_hook = hardwareMap.servo.get("left_hook");
        right_hook = hardwareMap.servo.get("right_hook");

        // Setting servos to the retracted position allowing them to move over the foundation lip
        left_hook .setPosition(lStored);
        right_hook.setPosition(rStored);

        // Waiting for the player to hit the play button on the drive station phone when autonomous starts
        waitForStart();
        armCollection.expand(true);
        sleep(4000);
        armCollection.expand(false);





    }


}
