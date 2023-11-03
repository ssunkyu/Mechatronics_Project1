#include <stdio.h>
#include <math.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <stdlib.h> // Library for dynamic memory allocation

#define SAMPLINGTIME 5 // Sampling time (ms)
// Gain settings
#define PGAIN 600
#define IGAIN 0.1
#define DGAIN 1.0

//# of GPIO pins
#define PULSE 5 // Should change
#define ENCODERA 17
#define ENCODERB 27
#define ENC2REDGEAR 217
#define MOTOR1 19
#define MOTOR2 26

int encA; // Signal of encA
int encB; // Signal of encB
int pulse; // Signal of pulse
int encoderPosition = 0; // Encoder position count
float redGearPosition = 0; // Actual rotation position of the red gear
float referencePosition = 0; // Desired reference position
float *refer_array = NULL; // Pointer of referfence rotation position 

unsigned int startTime = 0; // Start time of PID control
unsigned int checkTimeBefore = 0; // Previous check time for sampling
unsigned int checkTime = 0; // Current check time for sampling

float m; // Current motor input value
float m1; // Previous motor input value
float e; // Current error
float e1; // Previous error
float e2; // Error two samples ago

float ITAE = 0; // Integral of Time-weighted Absolute Error
float G1, G2, G3; // PID constants based on gain settings

int pulseChanged = 0; // Flag to indicate a change in pulse, 1 if a new pulse is detected
int pulse_n = 0; // Counter for the number of pulses processed

// This function is called when a rising edge is detected on the PULSE pin
void onPulseChange() {
    if (digitalRead(PULSE) == HIGH) {
        pulseChanged = 1; // Set the flag to indicate a pulse has been 
    }
}

void funcEncoderA()
{
    encA = digitalRead(ENCODERA);
    encB = digitalRead(ENCODERB);
    if (encA == HIGH)
    {
        if (encB == LOW) encoderPosition--;
        else encoderPosition++;
    }
    else
    {
        if (encB == LOW) encoderPosition++;
        else encoderPosition--;
    }
    redGearPosition = (float)encoderPosition / ENC2REDGEAR;
    e = referencePosition - redGearPosition;
    // printf("funcEncoderA() A: %d B: %d encPos: %d gearPos: %f\n", encA, encB, encoderPosition, redGearPosition);
}

void funcEncoderB()
{
    encA = digitalRead(ENCODERA);
    encB = digitalRead(ENCODERB);
    if (encB == HIGH)
    {
        if (encA == LOW) encoderPosition++;
        else encoderPosition--;
    }
    else
    {
        if (encA == LOW) encoderPosition--;
        else encoderPosition++;
    }
    redGearPosition = (float)encoderPosition / ENC2REDGEAR;
    e = referencePosition - redGearPosition;
    // printf("funcEncoderB() A: %d B: %d encPos: %d gearPos: %f\n",encA, encB, encoderPosition, redGearPosition);
}

void PID_CONTROL()
{
    // Record start time for PID control
    startTime = millis();
    checkTimeBefore = millis();
    // Calculate initial error
    e = referencePosition - redGearPosition;

    // Initialize m1, e1, e2
    m1 = 0;
    e1 = 0;
    e2 = 0;

    //GAIN EQUATIONS
    G1 = PGAIN + IGAIN*SAMPLINGTIME/1000.0 + DGAIN/SAMPLINGTIME*1000.0;
    G2 = -(PGAIN + 2*DGAIN/SAMPLINGTIME*1000.0);
    G3 = DGAIN/SAMPLINGTIME*1000.0;
    
    // Reset the pulse change flag
    pulseChanged = 0;
    
    while(!pulseChanged)
    {
        checkTime = millis();
        // Compute motor input using PID formula
        m = m1 + G1*e + G2*e1 + G3*e2;
        
        // Check if it's time for the next PID computation
        if (checkTime - checkTimeBefore >= SAMPLINGTIME){
            // Print position at fixed intervals for debugging
            if((checkTime-startTime)%100==0){
                printf("%f\n", redGearPosition);
            }
            // Apply control signal to motor
            if(e > 0){
                softPwmWrite(MOTOR2,m);
                softPwmWrite(MOTOR1,0);
            }
            else{
                softPwmWrite(MOTOR1,-m);
                softPwmWrite(MOTOR2,0);
            }
            // Record the time for this iteration
            checkTimeBefore = checkTime;
            // Update m1, e2, e1
            m1 = m;
            e2 = e1;
            e1 = e;
            // Update ITAE performance measure
            ITAE = ITAE + SAMPLINGTIME/1000.0 * (checkTime-startTime)/1000.0 * fabs(e);
        }
    }
}

int main(void)
{
    int n; // Number of trials
    printf("input how many n: ");
    scanf("%d", &n);

    // Array of reference positions for each trial
    refer_array = (float *)malloc(n * sizeof(float));
    if(refer_array == NULL) {
        printf("Fail!\n");
        return 1;
    }

    printf("Input int: ");
    for(int i = 0; i < n; i++) {
        scanf("%f", &refer_array[i]);
    }
    printf("Input Result: ");
    for(int i = 0; i < n; i++) {
        printf("%f ", refer_array[i]);
    }
    printf("\n");
   
    wiringPiSetupGpio();
    pinMode(ENCODERA, INPUT);
    pinMode(ENCODERB, INPUT);
    pinMode(PULSE, INPUT);

    softPwmCreate(MOTOR1, 0, 100);
    softPwmCreate(MOTOR2, 0, 100);

    wiringPiISR(ENCODERA, INT_EDGE_BOTH, funcEncoderA);
    wiringPiISR(ENCODERB, INT_EDGE_BOTH, funcEncoderB);
    // Set up an interrupt that calls onPulseChange when a rising edge is detected on the PULSE pin
    wiringPiISR(PULSE, INT_EDGE_RISING, onPulseChange);
    
    while(1)
    {
        // If the number of processed pulses equals the number of trails, exit the loop
		if (pulse_n==n) break;
		
        // Update the reference position from the refer_array
		referencePosition = refer_array[pulse_n];
        // If a pulse has been detected, perform PID control and increment the pulse counter
        if(pulseChanged)
        {
            PID_CONTROL();
            pulse_n++;
        }
    }
    // Free the dynamically allocated memory
    free(refer_array);
    softPwmWrite(MOTOR1, 0);
    delay(10); // Add delay to stop motor
    softPwmWrite(MOTOR2, 0);
    delay(10);
    printf("Finished!\n");

    printf("\nITAE: %.4f", ITAE);

    return 0;
}
