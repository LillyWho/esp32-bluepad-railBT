const uint8_t CVAR_1 = 1; //CHANGEME
const uint8_t CVAR_2 = 0;
const uint8_t CVAR_3 = 1;
const uint8_t CVAR_4 = 1;
const uint8_t CVAR_5 = 100;
const uint8_t CVAR_6 = 50;
/* The remaining standard NMRA CVARS are uninteresting for our purposes */

// CV 1 	Short Address (2-Digit Address) -- This is relevant if you want to run your engine as MU, otherwise leave this at default
// CV 2 	Start Volts (Vstart / "Boost") -- This offsets the motor duty cycle to something higher. We use a an integer between 0 and 255.
// CV 3 	Acceleration Rate
// CV 4 	Deceleration Rate
// CV 5 	Top Volts (Vhigh / Limit) -- Use this to throttle the duty cycle. All speed inputs will scale between CV 2 and this one. Integer between 0 and 255.