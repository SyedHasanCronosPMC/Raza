/*
Drone PID Simulator:

Syed Naqvi     250210060
Date: 02/05/2026

This program simulates a 1D vertical drone flight using a PID controller.
It features a main menu that allows the user to learn about the math or select a simulation level.
When a simulation is selected, an interactive arrow key menu allows the user to change the PID values.
A step response graph is then created and key stepinfo values are calculated and displayed.
The program has been coded to handle any invalid inputs.

Please Note: Gen AI was used to assist with brainstorming
and formatting the ASCII graph, but the final implementation
and structure are my own work.

Build (Windows): gcc openEndedC.c -lm -o drone.exe
This program is Windows-only because it uses <conio.h> for arrow-key input.
ANSI colors and cursor control require a VT100-capable terminal
(Windows Terminal or Windows 10+ console).
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <conio.h>


#define STEPS 100         // 10 seconds at 0.1s dt
#define DT 0.1
#define ROWS 15
#define COLS 60

#define KP_MAX 15.0
#define KI_MAX 10.0
#define KD_MAX 15.0
#define ALT_MAX 25.0

#define INTEGRAL_MAX 50.0   // anti-windup: keep Ki * integral from running away
#define THRUST_MAX 150.0    // motor saturation ceiling


typedef struct {
    double time;
    double altitude;
    double error;
    double thrust;
} FlightData;


void clearScreen();
void resetCursor();
void displayMenu();
void displayTutorial();
void displayMath();

int getValidMenuChoice();

void runLevel(int level, FlightData fLog[], int maxSteps, double dt);
void tunePID(double *kp, double *ki, double *kd, double *targetAlt, const char* levelName);
void runSimulation(double target, double kp, double ki, double kd, double mass, bool hasWind, FlightData log[], int maxSteps, double dt);
void drawStepResponse(FlightData log[], int maxSteps, double target, double kp, double ki, double kd);
void analyzeFlight(FlightData log[], int numSteps, double target, double dt);
void drawSlider(const char* label, double value, double maxVal, bool isSelected);
void saveFlightLog(const char* path, FlightData log[], int numSteps,
                   const char* levelName, double kp, double ki, double kd,
                   double targetAlt, double mass);


int main() {
    bool programRunning = true;

    FlightData flightLog[STEPS]; // array of structs, each 0.1 second is a new struct

    srand(time(NULL)); // for random value of wind

    do {
        displayMenu();

        // used to check if user has typed in a valid number
        int choice = getValidMenuChoice();

        switch (choice) {
            case 1: displayTutorial(); break;
            case 2: displayMath(); break;
            case 3: runLevel(1, flightLog, STEPS, DT); break; // Standard
            case 4: runLevel(2, flightLog, STEPS, DT); break; // Wind Tunnel
            case 5: runLevel(3, flightLog, STEPS, DT); break; // Heavy Payload
            case 6: programRunning = false; break;
            default: break;
        }
    } while (programRunning == true);
    clearScreen();
    printf("\033[36mPID System Terminated. Goodbye!\033[0m\n"); // ANSI code used for color
    return 0;
}


//asks the user to input a number between 1 and 6 for menu selection

int getValidMenuChoice() {
    int check = 0;              //variables reset if loop rerun
    bool isValidNum = false;
    int validNumber;

    while (isValidNum == false){
        printf("Enter your choice (1-6): ");
        check = scanf("%d", &validNumber);
        if (check != 1 || (validNumber < 1 || validNumber > 6)){
            printf("\033[31mPlease enter a whole number between 1 and 6\033[0m\n");
            while (getchar() != '\n');   // clears input buffer
        }
        else{
            isValidNum = true;
            while (getchar() != '\n');
        }
    }
    return validNumber;
}


void runLevel(int level, FlightData fLog[], int maxSteps, double dt) {
    double kp = 2.0, ki = 0.5, kd = 1.0, targetAlt = 10.0; // default values shown
    double mass = 1.0;
    bool hasWind = false; // disturbance to system
    char levelName[60];

    if (level == 1) {
        snprintf(levelName, sizeof(levelName), "Level 1: Standard Hover (Mass: 1kg, No Wind)");
        mass = 1.0; hasWind = false;
    } else if (level == 2) {
        snprintf(levelName, sizeof(levelName), "Level 2: The Wind Disturbance (Mass: 1kg, Gusts of Wind)");
        mass = 1.0; hasWind = true;
    } else if (level == 3) {
        snprintf(levelName, sizeof(levelName), "Level 3: Heavy Drone (Mass: 5kg, No Wind)");
        mass = 5.0; hasWind = false;
    }

    tunePID(&kp, &ki, &kd, &targetAlt, levelName);

    runSimulation(targetAlt, kp, ki, kd, mass, hasWind, fLog, maxSteps, dt);     // functions explained in detail in definition

    drawStepResponse(fLog, maxSteps, targetAlt, kp, ki, kd);

    analyzeFlight(fLog, maxSteps, targetAlt, dt);

    saveFlightLog("flight_log.csv", fLog, maxSteps, levelName, kp, ki, kd, targetAlt, mass);

    printf("\nPress Enter to return to Main Menu...\n");
    char temp[50];
    fgets(temp, sizeof(temp), stdin);         // ends reading when enter (\n) is pressed
}



void runSimulation(double target, double kp, double ki, double kd, double mass, bool hasWind, FlightData log[], int maxSteps, double dt) {
    double currentAlt = 0.0, velocity = 0.0, integral = 0.0;
    double prevError = target - currentAlt;
    double gravity = 9.81;
    double windForce = 0.0; // sustained between gust updates so wind acts continuously

    for (int step = 0; step < maxSteps; step++) {
        double error = target - currentAlt;
        integral = integral + (error * dt);     // sums up error for each 0.1s and stores total
        // Anti-windup: bound the integral so a long approach can't saturate Ki * integral
        if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
        if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

        double derivative = (error - prevError) / dt;

        double thrust = (kp * error) + (ki * integral) + (kd * derivative);   // u = P + I + D
        if (thrust < 0) thrust = 0;                          // drones can't pull down
        if (thrust > THRUST_MAX) thrust = THRUST_MAX;        // motor saturation

        if (hasWind && step % 20 == 0) {
            // Re-randomize the gust every 2s; held at this value until the next update
            windForce = ((double)rand() / RAND_MAX) * 8.0 - 4.0;
        }
        if (!hasWind) windForce = 0.0;

        double netForce = thrust - (mass * gravity) + windForce;
        double acceleration = netForce / mass;

        velocity = velocity + (acceleration * dt);    // first calculated to later calculate the current altitude
        currentAlt = currentAlt + (velocity * dt);

        if (currentAlt < 0.0) { currentAlt = 0.0; velocity = 0.0; } // can't go below ground level


        log[step].time = step * dt;
        log[step].altitude = currentAlt;
        log[step].error = error;
        log[step].thrust = thrust;
        prevError = error;
    }
}



// creating a step response graph
void drawStepResponse(FlightData log[], int maxSteps, double target, double kp, double ki, double kd) {
    char grid[ROWS][COLS];

    for (int r = 0; r < ROWS; r++) {   // creates an empty graph
        for (int c = 0; c < COLS; c++) { grid[r][c] = ' '; }
    }

    double maxAlt = target, minAlt = 0.0;
    for (int i = 0; i < maxSteps; i++) {
        if (log[i].altitude > maxAlt) { maxAlt = log[i].altitude; }
    }
    maxAlt += 2.0;
    double range = maxAlt - minAlt;   // allows us to fit curve to graph to make sure it is not cut off
    if (range == 0) range = 1.0;

    // target line
    int targetRow = ROWS - 1 - (int)(((target - minAlt) / range) * (ROWS - 1));   // finds the target row by calculating a percentage of the target relative to max
    for (int c = 0; c < COLS; c++) {                                              // height, it then flips the graph as row 0 starts from top and not 0 altitude
        if (targetRow >= 0 && targetRow < ROWS) { grid[targetRow][c] = '-'; }
    }

    // actual flight path
    for (int i = 0; i < maxSteps; i++) {
        int col = (int)(((double)i / maxSteps) * (COLS - 1));
        int row = ROWS - 1 - (int)(((log[i].altitude - minAlt) / range) * (ROWS - 1));     // each 0.1 s altitude is plotted onto the graph to create the curve
        if (row >= 0 && row < ROWS && col >= 0 && col < COLS) { grid[row][col] = '*'; }    // position on graph is same logic as target line
    }

    clearScreen();  // clears menu
    printf("\033[36m    Step Response    \033[0m\n");
    printf("\033[35mParameters Selected -> Kp: %.1f | Ki: %.1f | Kd: %.1f | Target: %.1f m\033[0m\n\n", kp, ki, kd, target);

    printf("Y-Axis: Altitude (m) | X-Axis: Time (10s)\n");
    printf("Legend: [\033[32m*\033[0m] Drone Path  [\033[33m-\033[0m] Target Altitude\n\n");

    for (int r = 0; r < ROWS; r++) {
        double rowValue = maxAlt - ((double)r / (ROWS - 1)) * range; // prints y axis values
        printf("%5.1f |", rowValue);
        for (int c = 0; c < COLS; c++) {
            if (grid[r][c] == '*') printf("\033[32m*\033[0m"); // green drone path
            else if (grid[r][c] == '-') printf("\033[33m-\033[0m"); // yellow target line
            else printf("%c", grid[r][c]);
        }
        printf("\n");
    }
    printf("      +");           // x axis
    for (int c = 0; c < COLS; c++) printf("-");
    printf("\n\n");
}

// Reports stepinfo: peak altitude, overshoot, rise time (10% to 90% of target),
// settling time (last exit from +/-2% band), and steady-state error averaged over
// the last 1 second so a single oscillation sample doesn't bias the result.
void analyzeFlight(FlightData log[], int numSteps, double target, double dt) {
    double maxReached = 0.0;

    for (int i = 0; i < numSteps; i++) {
        if (log[i].altitude > maxReached) { maxReached = log[i].altitude; } // peak value of path
    }

    double overshoot = maxReached - target;
    if (overshoot < 0) { overshoot = 0.0; }

    // Steady-state error: average the last 1 second of altitude
    int tail = (int)(1.0 / dt);
    if (tail > numSteps) tail = numSteps;
    double sumAlt = 0.0;
    for (int i = numSteps - tail; i < numSteps; i++) sumAlt += log[i].altitude;
    double finalAvg = sumAlt / tail;
    double steadyError = fabs(target - finalAvg);

    // Rise time: time from first crossing of 10% target to first crossing of 90% target
    double riseStart = -1.0, riseEnd = -1.0;
    for (int i = 0; i < numSteps; i++) {
        if (riseStart < 0 && log[i].altitude >= 0.1 * target) riseStart = log[i].time;
        if (log[i].altitude >= 0.9 * target) { riseEnd = log[i].time; break; }
    }
    double riseTime = (riseStart >= 0 && riseEnd >= 0) ? (riseEnd - riseStart) : -1.0;

    // Settling time: scan from the end and find the last sample outside the +/-2% band
    double band = 0.02 * target;
    if (band < 0.05) band = 0.05;          // floor for very small targets
    double settlingTime = -1.0;
    for (int i = numSteps - 1; i >= 0; i--) {
        if (fabs(log[i].altitude - target) > band) {
            if (i + 1 < numSteps) settlingTime = log[i + 1].time;
            break;
        }
    }

    printf("\033[36m--- Step Info ---\033[0m\n");
    printf("Max Altitude:    %.2f m\n", maxReached);
    printf("Overshoot:       %.2f m\n", overshoot);
    printf("Steady Error:    %.2f m  (avg over last 1.0s)\n", steadyError);
    if (riseTime >= 0)     printf("Rise Time:       %.2f s  (10%% -> 90%%)\n", riseTime);
    else                   printf("Rise Time:       not reached\n");
    if (settlingTime >= 0) printf("Settling Time:   %.2f s  (+/-2%% band)\n", settlingTime);
    else                   printf("Settling Time:   not settled\n");

    printf("Grade: ");
    if (steadyError > 1.5 || overshoot > 4.0) {
        printf("\033[31m Unstable - Needs Retuning\033[0m\n");
    } else if (steadyError > 0.5 || overshoot > 1.5) {
        printf("\033[33m Acceptable - Minor Oscillations / Error\033[0m\n");
    } else {
        printf("\033[32m Optimal - Good Control!\033[0m\n");
    }
}

void tunePID(double *kp, double *ki, double *kd, double *targetAlt, const char* levelName) {
    int selected = 0;
    bool tuning = true;

    int key;  // holds what is pressed in input

    clearScreen(); // clears menu

    do {
        resetCursor();

        printf("\033[36m=== %s ===\033[0m\n", levelName);
        printf("Use UP/DOWN arrows to select. LEFT/RIGHT arrows to adjust.  \n");
        printf("Press ENTER to run the simulation.                        \n\n");

        drawSlider("Proportional (Kp)", *kp, KP_MAX,  selected == 0);
        drawSlider("Integral (Ki)    ", *ki, KI_MAX,  selected == 1);
        drawSlider("Derivative (Kd)  ", *kd, KD_MAX,  selected == 2);
        drawSlider("Target Altitude  ", *targetAlt, ALT_MAX, selected == 3);

        if (selected == 4) { printf("\n> \033[32m[ Run Simulation ]\033[0m <          \n"); } // color when user on that selection
        else { printf("\n  [ Run Simulation ]            \n"); }

        key = _getch();
        if (key == 224 || key == 0) {  //sometimes arrow key gives a different code
            key = _getch();
            if (key == 72) { selected--; } //  selected decreases meaning it goes up a line
            if (key == 80) { selected++; }

            if (key == 75) { // left
                if (selected == 0 && *kp > 0) *kp -= 0.1;    // depends on which level you are
                if (selected == 1 && *ki > 0) *ki -= 0.1;
                if (selected == 2 && *kd > 0) *kd -= 0.1;
                if (selected == 3 && *targetAlt > 1) *targetAlt -= 1.0;
            }
            if (key == 77) { // right
                if (selected == 0 && *kp < KP_MAX) *kp += 0.1;
                if (selected == 1 && *ki < KI_MAX) *ki += 0.1;
                if (selected == 2 && *kd < KD_MAX) *kd += 0.1;
                if (selected == 3 && *targetAlt < ALT_MAX) *targetAlt += 1.0;
            }
        } else if (key == 13) { // enter removes the page and simulation begins
            if (selected == 4) { tuning = false; }
        }

        if (selected < 0) { selected = 4; }
        if (selected > 4) { selected = 0; }

    } while (tuning == true);
}

void drawSlider(const char* label, double value, double maxVal, bool isSelected) {
    int bWidth = 20; // bar width
    int pos = (int)((value / maxVal) * bWidth);
    if (pos >= bWidth) pos = bWidth - 1; // keep the O marker visible at the right edge

    if (isSelected) { printf("\033[33m-> \033[0m"); } else { printf("   "); }

    printf("%s [", label);
    for (int i = 0; i < bWidth; i++) {
        if (i < pos) printf("=");
        else if (i == pos) printf("\033[36mO\033[0m");
        else printf("-");
    }
    printf("] %5.1f   \n", value);
}

void displayMenu() {
    clearScreen();
    printf("\033[36m-----------------------------------------\n");
    printf("            Drone PID Simulator              \n");
    printf("-----------------------------------------\033[0m\n\n");
    printf("1. PID basics\n");
    printf("2. The math behind the code\n");
    printf("\033[32m3. Level 1: Standard Hover\033[0m\n");
    printf("\033[33m4. Level 2: The Wind Disturbance\033[0m\n");
    printf("\033[31m5. Level 3: Heavy Drone\033[0m\n");
    printf("6. Exit System\n\n");
}

void displayTutorial() {
    clearScreen();
    printf("\033[36m=== Intro to PID ===\033[0m\n\n");
    printf("\033[33m1. Proportional (Kp) \033[0m\n");
    printf("Increases the speed of control, however, the system would start to oscillate if Kp is too large\n\n");

    printf("\033[33m2. Integral (Ki) \033[0m\n");
    printf("Adds up past errors to give a hovering drone an extra push.\n\n");

    printf("\033[33m3. Deriviative (Kd) \033[0m\n");
    printf("Slows the drone down as it approaches the target to prevent overshoot.\n\n");

    printf("Press Enter to return...");
    char temp[50];
    fgets(temp, sizeof(temp), stdin);
}

void displayMath() {
    clearScreen();
    printf("\033[36m--- Math Used ---\033[0m\n\n");
    printf("1. Error = Target Altitude - Current Altitude\n");
    printf("2. Thrust (U) = (Kp * Error) + (Ki * Integral) + (Kd * Derivative)\n");
    printf("3. Net Force = Thrust - (Mass * Gravity) + Wind Disturbances\n");
    printf("4. Acceleration = Net Force / Mass\n");
    printf("5. Velocity = Velocity + (Acceleration * dt)\n");
    printf("6. Altitude = Altitude + (Velocity * dt)\n\n");
    printf("Press Enter to return...");
    char temp[50];
    fgets(temp, sizeof(temp), stdin);
}

void clearScreen() {
    #ifdef _WIN32
        system("cls");          // depending on which device
    #else
        system("clear");
    #endif
}

void resetCursor() {
    printf("\033[H");
}

// Writes the flight log to CSV. Header lines starting with '#' carry the run
// metadata; the data block is plain time,altitude,error,thrust columns so
// pandas, numpy, and Excel can read it directly.
void saveFlightLog(const char* path, FlightData log[], int numSteps,
                   const char* levelName, double kp, double ki, double kd,
                   double targetAlt, double mass) {
    FILE* fp = fopen(path, "w");
    if (!fp) {
        printf("\033[31mFailed to write %s\033[0m\n", path);
        return;
    }
    fprintf(fp, "# %s\n", levelName);
    fprintf(fp, "# Kp=%.2f Ki=%.2f Kd=%.2f Target=%.2fm Mass=%.2fkg\n",
            kp, ki, kd, targetAlt, mass);
    fprintf(fp, "time_s,altitude_m,error_m,thrust_N\n");
    for (int i = 0; i < numSteps; i++) {
        fprintf(fp, "%.2f,%.4f,%.4f,%.4f\n",
                log[i].time, log[i].altitude, log[i].error, log[i].thrust);
    }
    fclose(fp);
    printf("\033[36mFlight log saved to %s\033[0m\n", path);
}
