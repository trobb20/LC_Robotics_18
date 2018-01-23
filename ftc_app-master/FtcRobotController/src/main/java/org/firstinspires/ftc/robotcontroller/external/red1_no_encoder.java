package org.firstinspires.ftc.robotcontroller.external;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;


/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

@Autonomous(name="red1_no_encoder", group= "Autonomous")
public class red1_no_encoder extends LinearOpMode {

    // motor controllers
    DcMotorController driveController;
    DcMotorController armController;

    //servo controllers
    ServoController clawController;

    // motors
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorLift;

    //servos
    Servo topRight;
    Servo topLeft;
    Servo bottomRight;
    Servo bottomLeft;

    private ElapsedTime     runtime = new ElapsedTime();

    //set encoder stuff	
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
	static final double		REVOLUTIONS_R			= 1	;		//how many revolutions of the wheels can you make in one second at speed 1
	static final double		REVOLUTIONS_L			= 1	;	//^
	static final double		DIST_PER_SEC_R			= WHEEL_DIAMETER_INCHES*3.14*REVOLUTIONS_R;	//how many inches the wheels turn in one second at speed 1
	static final double		DIST_PER_SEC_L			= WHEEL_DIAMETER_INCHES*3.14*REVOLUTIONS_L;	//^
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.6;
	double openVal = 0.4;

    @Override


    public void runOpMode() {

        // motor controllers
        driveController = hardwareMap.dcMotorController.get("driveController");
        armController = hardwareMap.dcMotorController.get("armController");
        //servo controllers
        clawController=hardwareMap.servoController.get("clawController");

        //dcMotors
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLift = hardwareMap.dcMotor.get("motorLift");
        //servos
        topRight = hardwareMap.servo.get("topRight");
        topLeft = hardwareMap.servo.get("topLeft");
        bottomRight = hardwareMap.servo.get("bottomRight");
        bottomLeft = hardwareMap.servo.get("bottomLeft");
        //sensors

        //Set all powers to 0
        motorRight.setPower(0);
        motorLeft.setPower(0);
        motorLift.setPower(0);

        //close the claw
        topRight.setPosition(0);
        topLeft.setPosition(1);
        bottomRight.setPosition(1);
        bottomLeft.setPosition(0);

        //swap right direction
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*

        ====================================================================
                                DRIVING COMMANDS
        ====================================================================

        */

        //lift the block a bit
        motorLift.setPower(-0.5);
        sleep(800);
        motorLift.setPower(0);
		
		timeDrive(DRIVE_SPEED,22,22,0.5); //drive off platform
		
		timeDrive(TURN_SPEED, -6,6,0.5); //turn towards goal
		
		timeDrive(DRIVE_SPEED, 16,16,0.5); //drive towards goal
		
		dropBlock();
		
		timeDrive(DRIVE_SPEED, -8,-8,0.5); // pull back from goal.


        /*

        ====================================================================
                                         END
        ====================================================================

        */

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
	
	public void timeDrive(double speed, double leftInches, double rightInches, double wait){
		if (opModeIsActive()){
			
			double ratio = DIST_PER_SEC_L/DIST_PER_SEC_R; //ratio between their rates
			
			double speedL = speed*(leftInches/abs(leftInches));//this gives us direction 
			double speedR = speed*ratio*(rightInches/abs(rightInches));//make sure we scale the speeds so they take the same amount of time to go the same distance
			
			double rateL = abs(DIST_PER_SEC_L*speedL);
			/*
			double rateR = DIST_PER_SEC_R*speedR
			
			double timeL = leftInches/rateL
			double timeR = rightInches/rateR
			*/
			double time= abs(leftInches/rateL);
			
			telemetry.addData("Path1",  "Calculations Done. SPEED L:D %7d :%7d", speedL,  speedR);
            telemetry.addData("Path2",  "Runtime will be %7d",time);
            telemetry.update();
			
			sleep(2000); //sleep two seconds so drivers can see the telemetry
			
			//start driving
			runtime.reset();
			motorLeft.setPower(speedL);
			motorRight.setPower(speedR);
			
			while (opModeIsActive() && runtime.seconds()<time){
				remaining = runtime.seconds()-time;				//display the time remaining while the bot drives
				telemetry.addData("Path2",  "Time remaining: %7d",remaining);
				telemetry.update();
			}
			
			// Stop all motion;
            motorLeft.setPower(0);
            motorRight.setPower(0);
			
			sleep((long)wait*1000); //sleep before next move
		}
	}


    public void dropBlock(){
        //lower the block down a bit
        motorLift.setPower(0.5);
        sleep(500);
        motorLift.setPower(0);
        sleep(500);
        //let go of the block
		topRight.setPosition(0+openVal);
        topLeft.setPosition(1-openVal);
        bottomRight.setPosition(1-openVal);
        bottomLeft.setPosition(0+openVal);
		//lift the arm back up
        motorLift.setPower(-0.5);
        sleep(500);
        motorLift.setPower(0);
        sleep(500);
    }
}
