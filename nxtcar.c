/* btslave.c */ 
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* OSEK declarations */
DeclareCounter(SysTimerCnt);
DeclareTask(nxtcar);
DeclareTask(velocity); 
DeclareTask(IdleTask);
DeclareTask(MotorTask);
DeclareTask(StopTask);
DeclareEvent(fast);
DeclareEvent(slow);
DeclareEvent(forward);
DeclareEvent(backward);
DeclareEvent(stop);
DeclareEvent(release);
DeclareEvent(pause);
DeclareResource(res1);

/* below macro enables run-time Bluetooth connection */
#define RUNTIME_CONNECTION

/* LEJOS OSEK hooks */
void ecrobot_device_initialize()
{
#ifndef RUNTIME_CONNECTION
  ecrobot_init_bt_slave("LEJOS-OSEK");
#endif
}

void ecrobot_device_terminate()
{
  ecrobot_term_bt_connection();
}

/* LEJOS OSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
  StatusType ercd;

  ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
  if(ercd != E_OK)
  {
    ShutdownOS(ercd);
  }
}

// Define Float = 0 and Brake = 1
typedef enum _brake_mode {
	Float = 0,
	Brake
} brakeMode;

brakeMode mode2 = 0;

int speed = 0;
int flag = 0;

TASK(nxtcar)
{
  static U8 bt_receive_buf[32]; 

  display_clear(0);
  // read packet data from the master device  
  ecrobot_read_bt_packet(bt_receive_buf, 32);
  display_goto_xy(0, 1);
  display_int(bt_receive_buf[3], 0);

  if (bt_receive_buf[3] == 1) {
	SetEvent(MotorTask, forward); 	
  } else if (bt_receive_buf[3] == 2) {
	SetEvent(MotorTask, backward);
  } else {
	SetEvent(MotorTask, stop);
  }	
  display_goto_xy(0, 2);
  display_int(bt_receive_buf[4], 0);
  if(bt_receive_buf[4]==3) { //left
	nxt_motor_set_speed(NXT_PORT_A, -60, 0);	
  } else if(bt_receive_buf[4]==4) {//right
	nxt_motor_set_speed(NXT_PORT_A, 60, 0);
  } else {//stop
	nxt_motor_set_speed(NXT_PORT_A, 0, 0);
  }
  display_goto_xy(0, 3);
  display_int(bt_receive_buf[5], 0);
  if (bt_receive_buf[5] == 1) {
	SetEvent(velocity, fast); 
  } else if (bt_receive_buf[5] == 2) { 
        SetEvent(velocity, slow);
  }
  display_goto_xy(0, 4);
  display_int(bt_receive_buf[6], 0);
  if (bt_receive_buf[6] == 1) {
	mode2 = Brake;	
  } else if (bt_receive_buf[6] == 2){
 	mode2 = Float;
  }
  display_goto_xy(0, 5);

  display_int(bt_receive_buf[7], 0);
  if (bt_receive_buf[7] == 1) {	
	SetEvent(StopTask, pause);
	flag = 1;
  } else if (bt_receive_buf[7] == 2) {
	ecrobot_sound_tone(200, 100, 70);
  } else if (bt_receive_buf[7] == 0) {
	if (flag == 1) {
		SetEvent(StopTask, release);
		flag = 0;
	}
  }

  display_update();
  

  TerminateTask();

}

/* an extended task that waits for events fast or slow
 if the fast event happens, speed variable is set to 100
 if the slow event happens, speed variable is set to 80 */
TASK(velocity)
{
	EventMaskType wh;
	while(WaitEvent(fast | slow) == E_OK) {		
		GetEvent(velocity, &wh);
		if (wh & fast){
			speed = 100;
			ClearEvent(fast);
		} else if(wh & slow) {
			speed = 80;
			ClearEvent(slow);
		} 
	}

	TerminateTask();
}


/* an extended task that waits for events forward or backward or stop
 if the forward event happens, motor B and C start moving forward
 if the backward event happens, motor B and C start moving backward
 if the stop event happens, motor B and C stop (not moving) */
TASK(MotorTask)
{
	EventMaskType wh;
	while(WaitEvent(forward | backward | stop) == E_OK) {		
		GetEvent(MotorTask, &wh);
		if (wh & forward){
			GetResource(res1);
			nxt_motor_set_speed(NXT_PORT_B, (-1) * speed, 0);
			nxt_motor_set_speed(NXT_PORT_C, (-1) * speed, 0);
			ClearEvent(forward);
			ReleaseResource(res1);
		} else if(wh & backward) {
			GetResource(res1);
			nxt_motor_set_speed(NXT_PORT_B, speed, 0);
			nxt_motor_set_speed(NXT_PORT_C, speed, 0);
			ClearEvent(backward);
			ReleaseResource(res1);
		} else if (wh & stop) {
			GetResource(res1);
			nxt_motor_set_speed(NXT_PORT_B, 0, 0);
			nxt_motor_set_speed(NXT_PORT_C, 0, 0);
			ClearEvent(stop);
			ReleaseResource(res1);
		} 
	}

	TerminateTask();
}

/* an extended task that waits for events pause and release
 First of all, it waits for the event "pause" which gets the resource called res1 and let the 2 motors stop
 Next it waits for the event "release" which is called upon when the resource res1 needs to be released */

TASK(StopTask)
{
	while (1) {
		WaitEvent(pause);
		GetResource(res1);
		nxt_motor_set_speed(NXT_PORT_B, 0, mode2);
  		nxt_motor_set_speed(NXT_PORT_C, 0, mode2);				
		ClearEvent(pause);

		WaitEvent(release);
		ReleaseResource(res1);
		ClearEvent(release);
	}

	TerminateTask();
}

/* IdleTask */
TASK(IdleTask)
{
  static SINT bt_status = BT_NO_INIT;
  
  while(1)
  {  
#ifdef RUNTIME_CONNECTION
    ecrobot_init_bt_slave("LEJOS-OSEK");
#endif

    if (ecrobot_get_bt_status() == BT_STREAM && bt_status != BT_STREAM)
    {
      display_clear(0);
      display_goto_xy(0, 0);
      display_string("[BT]");
      display_update();
    }
    bt_status = ecrobot_get_bt_status();
  }	
}
