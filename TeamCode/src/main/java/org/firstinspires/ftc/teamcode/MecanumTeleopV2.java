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
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "MecanumTeleopV2", group = "Linear Opmode")
//@Disabled
public class MecanumTeleopV2 extends LinearOpMode {

    private HardwarePushbot_BucketBrigade robot = new HardwarePushbot_BucketBrigade();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ElapsedTime runtime = new ElapsedTime();

        robot.FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BlockArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BlockArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        robot.FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BlockArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftStick = gamepad1.left_stick_y;
            double rightStick = gamepad1.right_stick_y;
            double leftTrigger = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;
            double leftStick2 = gamepad2.left_stick_y;
            boolean Dpad_UP = gamepad2.dpad_up;
            boolean Dpad_DOWN = gamepad2.dpad_down;
            boolean A = gamepad2.a;
            boolean B = gamepad2.b;
            boolean X = gamepad2.x;
            boolean Y = gamepad2.y;

            if (leftTrigger > 0) {
                //left strafe
                robot.FrontLeftDrive.setPower(leftTrigger); //out
                robot.BackLeftDrive.setPower(-leftTrigger);  //out
                robot.FrontRightDrive.setPower(-leftTrigger); //in
                robot.BackRightDrive.setPower(leftTrigger); //in

            }
            if (rightTrigger > 0) {
                //right strafe
                robot.FrontLeftDrive.setPower(-rightTrigger); //out
                robot.BackLeftDrive.setPower(rightTrigger); //out
                robot.FrontRightDrive.setPower(rightTrigger); //in
                robot.BackRightDrive.setPower(-rightTrigger); //in
            }
            if (rightTrigger == 0 && leftTrigger == 0) {
                robot.FrontLeftDrive.setPower(leftStick);
                robot.BackLeftDrive.setPower(leftStick);
                robot.FrontRightDrive.setPower(rightStick);
                robot.BackRightDrive.setPower(rightStick);

            }
            if (leftStick2>.20) {
                robot.BlockArm.setPower(-.20);
               // robot.BlockArm.setPower(.20); //Deploy
            }
            if (leftStick2<-.20) {
                robot.BlockArm.setPower(.20);
                //robot.BlockArm.setPower(-.20); //Retract
            }
            if (leftStick2 > -.20 && leftStick2 < .20) { //safety l
                robot.BlockArm.setPower(-leftStick2);
                //robot.BlockArm.setPower(leftStick2);
            }


            /*if (Dpad_UP) { //deploy
                robot.BlockArm.setPower(.5);
            }
            while (gamepad2.dpad_up) {
                sleep(10);
            }
            robot.BlockArm.setPower(0);

            if (Dpad_DOWN) {            //retract
                robot.BlockArm.setPower(-0.55);
            }
            while (gamepad2.dpad_up) {
                sleep(10);
            }
            robot.BlockArm.setPower(0); */


            if (A) {    //in
                robot.Grabber.setPosition(.70);
                sleep(100);
            } else if (B) { //out
                robot.Grabber.setPosition(0);
                sleep(100);
            }

            if (X) {    //in
                robot.Arm.setPosition(.75);
                sleep(100);
            } else if (Y) { //out
                robot.Arm.setPosition(0);
                sleep(100);
            }
            // Show the elapsed game time and wheel power.
           /* telemetry.addData("Status", "Run Time: " + runtime].toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", FleftPower, FrightPower);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", BleftPower, BrightPower);
            telemetry.update();*/
            //Hello
        }
    }

    private boolean IsDpadUP() {
        if (gamepad2.dpad_up) {
            while (gamepad2.dpad_up) {
            }
            return true;
        } else {
            return false;
        }
    }

    private boolean IsDpadDOWN() {
        if (gamepad2.dpad_down) {
            while (gamepad2.dpad_down) {
            }
            return true;
        } else {
            return false;
        }
    }

    private boolean ButtonA() {
        if (gamepad2.a) {
            while (gamepad2.a) {
            }
            return true;
        } else {
            return false;
        }
    }

    private boolean ButtonB() {
        if (gamepad2.a) {
            while (gamepad2.a) {
            }
            return true;
        } else {
            return false;
        }
    }
}
/* if (-.1<left_joystik<.1){
    left.setpower(0)}
    if
     if (IsDpadUP()) {
                robot.BlockArm.setTargetPosition(360 + 1440); //1440 one revolution, 360 is a quarter of a revolution
                robot.BlockArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.BlockArm.setPower(0.25);
                while (robot.BlockArm.isBusy()) {
                    sleep(100);
                }
                robot.BlockArm.setPower(0);
                robot.BlockArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (Dpad_DOWN) {
                robot.BlockArm.setTargetPosition(0); //Going back to down
                robot.BlockArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.BlockArm.setPower(0.25);
                while (robot.BlockArm.isBusy()) {
                    sleep(100);
                }
                robot.BlockArm.setPower(0);
                robot.BlockArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (ButtonA()) {
                robot.Arm.setPosition(.5);
                sleep(100);
            }
            else if (ButtonB()){
                robot.Arm.setPosition(0);
                sleep(100);
            }


            // Show the elapsed game time and wheel power.
           /* telemetry.addData("Status", "Run Time: " + runtime].toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", FleftPower, FrightPower);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", BleftPower, BrightPower);
            telemetry.update();*/
//Hello
/*}
        }

private boolean IsDpadUP() {
        if (gamepad2.dpad_up) {
        while (gamepad2.dpad_up) {
        }
        return true;
        } else {
        return false;
        }
        }
private boolean IsDpadDOWN() {
        if (gamepad2.dpad_down) {
        while (gamepad2.dpad_down) {
        }
        return true;
        } else {
        return false;
        }
        }
private boolean ButtonA() {
        if (gamepad2.a) {
        while (gamepad2.a) {
        }
        return true;
        } else {
        return false;
        }
        }
private boolean ButtonB() {
        if (gamepad2.a) {
        while (gamepad2.a) {
        }
        return true;
        } else {
        return false;
        }
 */