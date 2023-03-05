// Kurtarma işlemi için fonksiyonlar
#include "rescue.h"


// Function to check if emergency conditions are met
int check_emergency_conditions(float Kalman_x, float Kalman_y) {
    if (Kalman_x > X_THRESHOLD || Kalman_y > Y_THRESHOLD) {
        return 1;
    }
    return 0;
}

// Function to check if recovery process needs to be initiated
int check_recovery_conditions(float acc_xyz ) { //
    if (acc_xyz == 0) {
        return 1;
    }
    return 0;
}

// Function to open parachute
void open_parachute(int parachute_num) {
    printf("Opening Parachute %d\n", parachute_num); //girilen sayıya göre açıcagı parasute karar vericek
}

// Function to start the rescue process
void start_rescue_process() {
    printf("Starting Rescue Process\n");
    open_parachute(1);
    open_parachute(2); //bu durumda parasutu hemen acmak mantıklı olmayabilir ama simdilik boyle
}
