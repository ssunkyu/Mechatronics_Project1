#include <stdio.h>
#include <math.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <stdlib.h>
#include <string.h>

#define SAMPLINGTIME 5 // Sampling time (ms)
#define LOOPTIME 5000 // Loop time (ms), Drive motor for 5 seconds
// Gain Settings
#define PGAIN 1000.0
#define IGAIN 0.1
#define DGAIN 1.0

// # of GPIO Pins 
#define ENCODERA 17
#define ENCODERB 27
#define ENC2REDGEAR 217
#define MOTOR1 19
#define MOTOR2 26

#define ARRAY_SIZE 10000 // Size of array (row)
#define NUM_COLUMNS 2 // Size of array (col, time & position)
#define DAQ_TIME 10000 // Data logging for 10s

int n; // number of trails

int encA; // signal of encA
int encB; // signal of encB
int pulse; // signal of pulse
int encoderPosition = 0; // Position of Encoder
float redGearPosition = 0; // Rotation Position of Motor
float referencePosition = 0; // Reference Rotation Position
float *refer_array = NULL;

unsigned int startTime = 0;
unsigned int checkTimeBefore = 0;
unsigned int checkTime = 0;

float m; // Input to Motor
float m1; // Previous Input to Motor 
float e; // Present Step Error
float e1; // Previous Step Error
float e2; // Two Steps back Step Error

int dataIndex = 0;
float dataArray[ARRAY_SIZE][NUM_COLUMNS]; // 2D array (rows * cols)

float ITAE; // number of Rotation;

float G1, G2, G3; // Constant values of results of GAIN calculation

// Save time and Position of motor
void updateDataArray()
{
    dataArray[dataIndex][0] = (float)(checkTime - startTime) / 1000.0; 
    dataArray[dataIndex][1] = redGearPosition;
    dataIndex++;
}

// Plot the result
void plotGraph()
{
    FILE *gnuplot = popen("gnuplot -persistent", "w"); // Create a pipe
    
    fprintf(gnuplot, "set yrange [0:%lf]\n",referencePosition*1.2);
    fprintf(gnuplot, "set xlabel 'Time [s]'\n"); // Set a x label
    fprintf(gnuplot, "set ylabel 'Motor position [r]'\n"); // Set a y label
    fprintf(gnuplot, "plot '-' with lines title 'Position', %f with lines lt 2 title 'Ref'\n",referencePosition);
    for (int i = 0; i < dataIndex; ++i) {
        fprintf(gnuplot, "%f %f\n", dataArray[i][0], dataArray[i][1]);
        // Plot motor position
    }
    fprintf(gnuplot, "e\n");
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
    
    G1 = PGAIN + IGAIN*SAMPLINGTIME/1000.0 + DGAIN/SAMPLINGTIME*1000.0;
    G2 = -(PGAIN + 2*DGAIN/SAMPLINGTIME*1000.0);
    G3 = DGAIN/SAMPLINGTIME*1000.0;

    while(1){
        checkTime = millis();
        if ((checkTime-startTime >= LOOPTIME)){
            break;
        }
        m = m1 + G1*e + G2*e1 + G3*e2;
        // printf("aa: %f\n", m);
        if (checkTime - checkTimeBefore >= SAMPLINGTIME){
            //printf("loop time: %d ms, after init: %d ms\n", checkTime-checkTimeBefore,checkTime-startTime);
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
            updateDataArray();
            m1 = m;
            e1 = e;
            e2 = e1;
            ITAE = ITAE + SAMPLINGTIME/1000.0 * (checkTime-startTime)/1000.0 * fabs(e);
            
        }
    }
}

int main(void)
{
    char filename[100];
    char filepath[200];
    FILE* file;
    sprintf(filename, "%.1f_%.1f_%.1f", PGAIN, IGAIN, DGAIN);
    sprintf(filepath, "/home/pi/Mechatronics/csv/%s.csv", filename);
    file = fopen(filepath, "w+");
    
    referencePosition = 8.0;
   
    wiringPiSetupGpio();
    pinMode(ENCODERA, INPUT);
    pinMode(ENCODERB, INPUT);

    softPwmCreate(MOTOR1, 0, 100);
    softPwmCreate(MOTOR2, 0, 100);

    wiringPiISR(ENCODERA, INT_EDGE_BOTH, funcEncoderB);
    wiringPiISR(ENCODERB, INT_EDGE_BOTH, funcEncoderA);

    PID_CONTROL();
    
    softPwmWrite(MOTOR1, 0);
    softPwmWrite(MOTOR2, 0);
    printf("Finished!\n");
    
    printf("\nITAE: %.4f", ITAE);
    
    for (int i=0;i<dataIndex;i++)
    {
        fprintf(file, "%.3f,%.3f\n", dataArray[i][0], dataArray[i][1]);
    }
    fclose(file);
    
    plotGraph();
    
    return 0;
}
