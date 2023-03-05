#ifndef rescue_h
#define rescue_h

#define MIN_ALT 100    // Kurtarma işleminin başlaması için gereken min. irtifa
#define X_THRESHOLD 45     // Acil durumu başlatmak için x acisinin deger esigi
#define Y_THRESHOLD 45    // Acil durumu başlatmak için y acisinin deger esigi
#define SEC_ALT 1000      // 2. parasutu açması gereken irtifa
int parachute_opened = 0;  // parasut acılma flag
int recovery_started = 0;  //kurtarma flag
int emergency_init = 0; //acil durum flag

//emegency_functions
int check_emergency_conditions(float Kalman_x, float Kalman_y);
int check_recovery_conditions(float acc_xyz );
void open_parachute(int parachute_num);
void start_rescue_process();


#endif
