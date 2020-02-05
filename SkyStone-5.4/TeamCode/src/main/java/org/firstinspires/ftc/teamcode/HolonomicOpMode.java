/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *//*


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

*/
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *//*


@TeleOp(name="Holonomic OpMode", group="Iterative Opmode")
//@Disabled
public class HolonomicOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor  FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor, IntakeLeftMotor, IntakeRightMotor, ScissorLiftMotorLeft, ScissorLiftMotorRight;
    private Servo IntakePulley, left_hook, right_hook;

    HolonomicDrive holonomicDrive;
    ScissorLift scissorLift;
    Intake_Systems intake_systems;
    BotServos bot_servo;

    */
/*
     * Code to run ONCE when the driver hits INIT
     *//*

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FrontRightMotor  = hardwareMap.get(DcMotor.class, "front_right_drive");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        BackRightMotor  = hardwareMap.get(DcMotor.class, "back_right_drive");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
        IntakeLeftMotor = hardwareMap.get(DcMotor.class, "left_intake");
        IntakeRightMotor = hardwareMap.get(DcMotor.class, "right_intake");
        ScissorLiftMotorLeft =  hardwareMap.get(DcMotor.class, "scissor_left");
        ScissorLiftMotorRight =  hardwareMap.get(DcMotor.class, "scissor_right");
        IntakePulley = hardwareMap.servo.get("intake_pulley");
        left_hook = hardwareMap.servo.get("left_hook");
        right_hook = hardwareMap.servo.get("right_hook");

        holonomicDrive = new HolonomicDrive(FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor);
        scissorLift = new ScissorLift(ScissorLiftMotorLeft, ScissorLiftMotorRight);
        intake_systems = new Intake_Systems(IntakeRightMotor, IntakeLeftMotor, IntakePulley);
        bot_servo = new BotServos(left_hook, right_hook);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    */
/*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     *//*

    @Override
    public void init_loop() {
    }

    */
/*
     * Code to run ONCE when the driver hits PLAY
     *//*

    @Override
    public void start() {
        runtime.reset();
    }

    */
/*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     *//*

    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double z = gamepad1.right_stick_x;



        double y2 = gamepad2.left_stick_y;
        boolean collect1 = gamepad1.a;
        boolean deploy1 = gamepad1.b;
        boolean collect2 = gamepad2.a;
        boolean deploy2 = gamepad2.b;
        boolean activate = gamepad2.right_bumper;
        boolean reset = gamepad2.left_bumper;
        boolean rbl = gamepad1.right_bumper;
        boolean lb1 = gamepad1.left_bumper;
        boolean button_y1 = gamepad1.y;
        boolean button_y2 = gamepad2.y;
        boolean button_x1 = gamepad1.x;
        boolean button_x2 = gamepad2.x;
        */
/*boolean scissorUp = gamepad2.dpad_up;
        boolean scissorDown = gamepad2.dpad_down;*//*



        holonomicDrive.teleopDrive(x,y,z);
        intake_systems.intakeTele(collect1, deploy1, collect2, deploy2);
        intake_systems.pullBackCollectionArms(button_x1, button_x2);
        intake_systems.releaseCollectionArms(button_y1);
        scissorLift.LiftControl(y2);
        bot_servo.retract(lb1, reset);
        bot_servo.activate(rbl, activate);
        //scissorLift.LiftMovement(scissorUp, scissorDown, reset, activate);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    */
/*
     * Code to run ONCE after the driver hits STOP
     *//*

    @Override
    public void stop() {
    }

}
*/
