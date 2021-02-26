// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{

//Motors defined here
	public static final int MOTOR_TOP_LEFT_ID = 1;
	public static final int MOTOR_BOTTOM_LEFT_ID = 2;
	public static final int MOTOR_TOP_RIGHT_ID = 3;
    public static final int MOTOR_BOTTON_RIGHT_ID = 4;
//-----------------------------------------------------    
//Buttons/Axis defined here
	public static final int XBUTTON = 1;
	public static final int YBUTTON = 4;
	public static final int ABUTTON = 2;
	public static final int BBUTTON = 3;
	public static final int RBBUTTON = 6;
	public static final int LBBUTTON = 5;
	public static final int rtBUTTON = 8;
	public static final int ltBUTTON = 7;
	public static final int RX_AXIS = 2;
	public static final int LY_AXIS = 1;
//-----------------------------------------------------

//Controllers define here
	public static final int PILOT_DRIVER_CONTROLLER = 0;
	public static final double DT_HG = 1;
	public static final double DT_HGT = 0.9;
	public static final double DT_LG = 0.8;
	public static final double DT_LGT = 0.65;
	public static final double FEET_TO_ROT_UNITS = (2048) * (50 / 24) * (2 / Math.PI);//(Ticks per rev) * (gear ratio) / (rev/ft)  c=6pi in = pi/2 ft
	public static final int kTimeoutMS = 30;
	public static final int kPIDLoopIdx = 0;
	public static final int SHOOT_MOTOR_LEFT_ID = 5;
	public static final int SHOOT_MOTOR_RIGHT_ID = 6;
	public static final int kTimeMS = 0;
	public static final int kSlotIdx = 0;
	public static final int ADJUST_ANGLE_MOTOR_LEFT = 11; // spark #3
	public static final int ADJUST_ANGLE_MOTOR_RIGHT = 9; // spark #4
	public static final double FEEDFORWARD = 0.009;
	public static final double ANGLE_MOTOR_SPEED = 0.1;
	public static final double SHOOTER_MOTOR_SPEED = 0.7;
	public static final int INTAKE_MOTOR_ID = 10; // spark #1
	public static final int INDEXER_MOTOR_1 = 7; //Left
	public static final int INDEXER_MOTOR_2 = 8; //Right
	public static final int SPIN_UP_MOTOR = 12; // spark #2
	public static final double INDEXER_SPEED = 1;
	public static final double SPIN_UP_SPEED = 0.8;
	public static final double INTAKE_BALL_SPEED = 0.9;
	public static final double allowableCloseLoopError = 0;

//------------------------------------------------------
}
