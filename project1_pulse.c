#include <stdio.h>
#include <math.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <termio.h>
#include <unistd.h>
#include <stdlib.h>

#define SAMPLINGTIME 0.01  // Sampling time (s)
#define LOOPTIME 5000 // Loop time (ms)
#define ENCODERA 17
#define ENCODERB 27
#define ENC2REDGEAR 217
#define PULSE 5 // should change

#define MOTOR1 19
#define MOTOR2 26

#define PGAIN 1000
#define IGAIN 8
#define DGAIN 1

int n; // number of trails
int encA; // signal of encA
int encB; // signal of encB
int pulse; // signal of pulse
int encoderPosition = 0; // Position of Encoder
float redGearPosition = 0; // Actual Rotation Position
float referencePosition = 0; // Reference Rotation Position
float *refer_array = NULL;

unsigned int startTime = 0;
unsigned int checkTimeBefore = 0;
unsigned int checkTime = 0;

float m; // Input to Motor
float m1; // Prevvious Input to Motor 
float e; // Present Step Error
float e1; // Previous Step Error
float e2; // Two Steps back Step Error

float ITAE; // number of Rotation;
float G1, G2, G3; // Constant values of results of GAIN calculation

int pulseChanged = 0;

void onPulseChange() {
    if (digitalRead(PULSE) == HIGH) {
        pulseChanged = 1;
    }
}

void funcEncoderA()
{
    encA = digitalRead(ENCODERA);
    encB = digitalRead(ENCODERB);
    if (encA == HIGH)
    {
        if (encB == LOW) encoderPosition++;
        else encoderPosition--;
    }
    else
    {
        if (encB == LOW) encoderPosition--;
        else encoderPosition++;
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
        if (encA == LOW) encoderPosition--;
        else encoderPosition++;
    }

    else
    {
        if (encA == LOW) encoderPosition++;
        else encoderPosition--;
    }

    redGearPosition = (float)encoderPosition / ENC2REDGEAR;
    e = referencePosition - redGearPosition;
    // printf("funcEncoderB() A: %d B: %d encPos: %d gearPos: %f\n",encA, encB, encoderPosition, redGearPosition);
}

void PID_CONTROL(){
    startTime = millis();
    checkTimeBefore = millis();
    e = referencePosition - redGearPosition;

    m1 = 0;
    e1 = 0;
    e2 = 0;

    //GAIN EQUATIONS
    G1 = PGAIN + IGAIN*SAMPLINGTIME + DGAIN/SAMPLINGTIME;
    G2 = -(PGAIN + 2*DGAIN/SAMPLINGTIME);
    G3 = DGAIN/SAMPLINGTIME;

    while(1)
    {
		pulseChanged = 0;
        checkTime = millis();
        if (pulseChanged)
        {
            pulseChanged = 0; // Reset the flag
            break;
        }

        m = m1 + G1*e + G2*e1 + G3*e2;
        // printf("aa: %f\n", m);

        if (checkTime - checkTimeBefore > SAMPLINGTIME){
            if((checkTime-startTime)%100==0){
                printf("%f\n", redGearPosition);
            }

            if(e > 0){
                softPwmWrite(MOTOR2,m);
                softPwmWrite(MOTOR1,0);
            }

            else{
                softPwmWrite(MOTOR1,-m);
                softPwmWrite(MOTOR2,0);
            }

            checkTimeBefore = checkTime;
            m1 = m;
            e1 = e;
            e2 = e1;

            ITAE = ITAE + SAMPLINGTIME * (checkTime-startTime)/1000.0 * fabs(e);
        }
    }
}

int main(void)
{
    int n;
    printf("input how many n: ");
    scanf("%d", &n);

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

    wiringPiISR(ENCODERA, INT_EDGE_BOTH, funcEncoderB);
    wiringPiISR(ENCODERB, INT_EDGE_BOTH, funcEncoderA);
    wiringPiISR(PULSE, INT_EDGE_RISING, onPulseChange);

    int i = 0;
    pulse = LOW;

    while(1)
    {
		if (i ==n) break;
		
		referencePosition = refer_array[i];
        pulse = digitalRead(PULSE);
        if(pulseChanged)
        {
            PID_CONTROL();
            i +=1;
            pulseChanged = 1;
        }
    }

    softPwmWrite(MOTOR1, 0);
    softPwmWrite(MOTOR2, 0);
    printf("Finished!\n");

    printf("\nITAE: %.4f", ITAE);

    return 0;
}

