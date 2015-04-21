
/*	PD-control for ResQ.PL 
/		Author:	Adnan Berberovic
/				Robert Oprea
/		Date: 	2015-03-30
*/

// Y = PD*G/(1+PD*G) * R

// Reference value = 0
// Error value e = r-y.
// We want to regulate the vehicle to drive in the middle of the corridor,
// So the reference value r should correspond to 20 cm.

int reference_ = 1;
int offset_;
int prev_error_ = 0;
int time_diff_;
int time_new_ = 0;
int prev_time_ = 0;
int angle_;
int K_d = 1; // K (constant) for D regulation
int K_p = 1; // K (proportion) för P regulation

// P-control function
int P_Control()
{
	int newSignal_P = K_p*(reference_ - offset_); // Calculate regulated value

	return newSignal_P;
}

// D-control function
int D_control()
{
	// time_new_ = TCNT0; // Retrieve the current time
	time_diff_ = time_new - prev_time_;
	int newSignal_D = K_d*(reference_ - offset_ - prev_error_)/time_diff_;
	prev_error_ = reference_ - offset_; // Save the current error for next computation
	prev_time_ = time_new_; // Save the current time for..
	return newSignal_D;
}

// Total control.
int PD_Control(int offset)
{
	offset_ = offset;
	int newSignal = angle*(P_Control()+D_control());

	return newSignal;
}