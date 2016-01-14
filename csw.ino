// Collaborative-smart-watering
// Arduino code for data acquisition and control through control() function
// December 2015

#include <FreqMeasure.h>
#include <avr/wdt.h>
#include <math.h>
#include <CmdMessenger.h>  // CmdMessenger

                                                                            //GLOBAL DEFINITIONS
int sondas[] = {5, 7, 6, 4, 2, 1, 0, 3}, uln[] = {11, 10, 9, 7, 6, 5, 4};	  //Probes pins in mux and digital pins for ULN control
float vector[]={0,0,0,0,0,0,0,0,0};                                         //Array that holds the probes data in float
float vector_old[]={1,1,1,1,1,1,1,1,1};                                     //Array that holds the last probes data in float
const int n_matrix=24;                                                      //control matrix [matrix] dimention
const int n_log_matrix=10;                                                  //log matrix [log_matrix] dimention
volatile unsigned int matrix[n_matrix][5];                                  //control matrix
volatile unsigned int log_matrix[n_log_matrix][5];                          //log matrix
volatile unsigned long time_matrix[n_matrix];                               //1D Array to holt timestamp in unsignef long int for the control matrix
volatile unsigned long time_log_matrix[n_log_matrix];                       //1D Array to holt timestamp in unsignef long int for the log matrix
int baloico = 3, input = 8, botao = 2; 																			//pins for inputs
int serial0 = A5, serial1 = A4, serial2 = A3;																//pins for MUX selection
int flag_rega = 0, aquisition_done = 0, count = 0, i = 0;                   //flags and iterators
int luz = 0;                                                                //luminosity
volatile int flag_wdt = 0, f_save=0, i_matrix=-1, i_log_matrix=0, first_run=1;//flags and iterators
float frequency = 0;
double sum = 0;
volatile unsigned long time = 0;
volatile int count_rega = 0, f_t1=0;
int overflowst2 = 0;
int dt_control=1;
int error=0;
int out, integral=0;

///////////////USER INTERFACE//////////////////////////
// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger = CmdMessenger(Serial);

// This is the list of recognized commands. These can be commands that can either be sent or received. 
// In order to receive, attach a callback function to these events
enum
{
  NADA            ,                                                  //to avoid defalult commands
  Log              ,                                                 //command to print the log matrix
  Control           ,                                                //command to print the control matrix
};

// Callbacks define on which received commands we take action
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(NADA, OnNADA);
  cmdMessenger.attach(Log, OnLog);
  cmdMessenger.attach(Control, OnControl);
}


void OnNADA()    //Default does nothing
{
}
void OnLog()                                  //PRINT LOG MATRIX
{
  Serial.print("-----------------LOG t=");
  Serial.print(time);
  Serial.println("------------------");
  int j;
  int jj;                                     //i_log_matrix equivalent
  for(jj=0;jj<n_log_matrix;jj++){             //runs log_matrix lines
    Serial.print(time_log_matrix[jj]);        //print time
    Serial.print("\t");
    for(j=0;j<5;j++){                         //print the values
      Serial.print(log_matrix[jj][j]);
      Serial.print("\t");
   }
   Serial.print("\n");
  }
  Serial.println("-----------------------------------------");
}
void OnControl()                                //PRINT CONTROL MATRIX
{
  Serial.print("--------------Control Log t=");
  Serial.print(time);
  Serial.println("-----------");
  int j;
  int jj;                                       //i_matrix equivalent
  for(jj=0;jj<n_matrix;jj++){                   //runs matrix lines
    Serial.print(time_matrix[jj]);
    Serial.print("\t");
    for(j=0;j<5;j++){
      Serial.print(matrix[jj][j]);
      Serial.print("\t");
   }
   Serial.print("\n");
  }
  Serial.println("---------------------------------------");
}

/* TMR2 Overflow interrupt vector */
ISR(TIMER2_OVF_vect) {   
  overflowst2++;                          // increment overflow count
  if (overflowst2 == 62) {
    flag_wdt = 1;                         //Set timeout flag
    overflowst2 = 0;                      //clear count
  }
}

//Reset Timer 2 - timeout 
void t2_reset() {
  overflowst2 = 0;            //clear number of timer overflows
  TCNT2  = 0;                 //clear timer counter
  flag_wdt=0;                 //clear timeout flag
}

//ISR to count the number of cups 
void Count_ISR() {
  count_rega += 1;
}

//ISR that detect button press 
void button_ISR() {
  count_rega=0;       //clears number of cups
  flag_rega = 1;      //raises flag
}

/* TMR1 OC interrupt vector */
ISR(TIMER1_COMPA_vect){
  if(first_run==0)                  //does not increment in the fisrt interrupt so that t=0
    time+=4;                        //increment time
  else
    first_run=0;                    //In first run toggle flag        
  if(f_t1 == 0 && time%8==0)        //every 8 sec.
  {
    f_t1 = 1;                       //raise flag to measure
    if(time%3600==0 && time!=0){    //every hour
      f_save=1;                     //raise flag to save data in control matrix
      i_matrix+=1;                  //increments matrix line
    }
  }
}

//APPLIES CALIBRATION 
void contas(){
//conductividade

  int calib1[]={5712,2000};  //xmin, ymin
  int calib2[]={5449,2000};  //xmin, ymin

  float k1,k2;
  k1=1./0.492354*(vector[2]-calib1[0])+calib1[1];
  k2=1./0.475907*(vector[4]-calib2[0])+calib2[1]; 


 //minimum detectability conditions
  if(k1<=1000)        
    vector[2]=1000;
    else
    vector[2]=int(k1);
 

  if(k2<=1000)
    vector[4]=1000;
    else
    vector[4]=int(k2);

  
 //temperatura

  float a1=-2.6289e5;
  float b1=7.59e-14;
  float c1=7245.9;
  float a2=-2.9721e5;
  float b2=1.2225e-14;
  float c2=7814.8;


  vector[1]=int(a1/(log(vector[1]/b1)) +c1);
  vector[3]=int(a2/(log(vector[3]/b2)) +c2);
  
}

//ROUTINE TO WATTER PLANTS
void rega(int n_copos) {                                                //number of cups (for each vase) as input
  detachInterrupt(digitalPinToInterrupt(botao));                        //renders button useless
  digitalWrite(uln[0], HIGH);                                           //activates water valve
  while (count_rega < 2 * n_copos) {                                    //waits for the wattering to finish
    1;
  }
  digitalWrite(uln[0], LOW);                                            //turns of valve
  count_rega = 0;                                                       //clears number of cups
  attachInterrupt(digitalPinToInterrupt(botao), button_ISR, FALLING);   //turns button interrupt on
}

//MUX INPUT SELECTION
void pin_sel(int num) {

  digitalWrite(serial2, LOW);	 //reset adress pins
  digitalWrite(serial1, LOW);
  digitalWrite(serial0, LOW);

  //Bitwise reading of the adress and pin activation
  if ( (num & 0b1) != 0 ) {
    digitalWrite(serial0, HIGH);
  }

  if ( (num & 0b10) != 0 ) {
    digitalWrite(serial1, HIGH);
  }

  if ( (num & 0b100) != 0 ) {
    digitalWrite(serial2, HIGH);
  }

}

//ROUTINE TO READ PROBE
void aquire() {
  aquisition_done = 0;                                          //clears flag
  FreqMeasure.begin();                                          //begins frequency measurement
  while (aquisition_done == 0) {                                
    if (FreqMeasure.available()) {                              //If there are availible frequencies to measure
      t2_reset();                                               //resets timeout timer
      // average over (300) measures
      sum = sum + FreqMeasure.read();                           
      count = count + 1;                                          
      if (count > 298) {                                        
        frequency = FreqMeasure.countToFrequency(sum / count);  //average
        vector[i+1]=frequency;                                  //saves result in float vector
        FreqMeasure.end();                                      //reset buffer with number of cycles
        i += 1;                                                 //next probe
        if (i == 8) {                                           //all probes measured
          i = 0;                                                //resets counter 
          aquisition_done = 1;                                  //sets flag
        }
        pin_sel(sondas[i]);                                     //selects next probe
        sum = 0;
        count = 0;
        FreqMeasure.begin();                                    //resets frequency measurement
      }
    }

    if (flag_wdt == 1) {                                        //If timeout
      t2_reset();                                               //resets timer
      vector[i+1]=0;                                            //sets value to 0
      FreqMeasure.end();                                        //clears buffer
      i += 1;                                                   //next probe
      if (i == 8) {                                             //all probes measured
        i = 0;
        aquisition_done = 1;
      }
      pin_sel(sondas[i]);                                       //select next probe
      sum = 0;                                                  //reset
      count = 0;
      flag_wdt = 0;
      FreqMeasure.begin();
    }

  }
  FreqMeasure.end();                  //stop measurement
  luz = analogRead(A0);               //reads luminosity
  vector[0]=luz;
}
//Print vector (not used)
void imprime(){
  int j=0;
  Serial.print(time);
  Serial.print(" \t");
  for(j=0;j<5;j++){
    Serial.print(vector[j]);
    Serial.print("\t");
   }
  Serial.print("\n");
}

//CONTROL ROUTINE
void control(){
  int j,copos;
  int setpoint =9000;                                   //uS/m
  float kp=0.405/30.;
  float ti=8.3333;
  int conductiv_measured;
  
  Serial.println("--------CONTROL--------");
  long int avg[]={0,0,0,0};
  for(j=0;j<4;j++){                                     //averages probes conductivities
    for(i_matrix=0;i_matrix<n_matrix;i_matrix++){
      avg[j]+=matrix[i_matrix][j+1];
      Serial.print(matrix[i_matrix][j+1]);
      Serial.print("\t");
   }
   avg[j]=avg[j]/n_matrix;
   Serial.print(avg[j]);
   Serial.print("\n");
  }
  i_matrix=-1;

  conductiv_measured= (avg[1]+avg[3])/2;                //averages both vases
  error=setpoint-conductiv_measured;                    //calculates error
  integral+=error*dt_control;                           //calculates integral contribution

  out= kp*(error+integral/ti);                          //computes responce
  //print control values
  Serial.print("\nPI response: ");
  Serial.print(out);
  Serial.print("\t error: ");
  Serial.print(error);
  Serial.print("\t integral: ");
  Serial.println(integral);
  Serial.println("\n------------------------");

  copos=int(out);                                       //responce in integer
  //limits to the actuator responce
  if (copos>30)
    copos=30;
  if (copos<0)
    copos=0;

  rega(copos);                                          //actuation
}

//SAVES VECTOR TO  CONTROL MATRIX
void save_vector(){
  int j;
  time_matrix[i_matrix]=time;
  for(j=0;j<5;j++){
    matrix[i_matrix][j]=vector[j];
  }
}
//SAVES VECTOR TO  LOG MATRIX
void save_log_vector(){                                       //If the new data has a significant variation saves it in log_matrix
  int j;
  int jj;
  int save;                                                 
  save=0;
  j=1;
  for(j=1;j<5;j++){                                          //runs values and compares with the old value
    if(vector_old[j]!=0 && abs( (vector[j]-vector_old[j])/vector_old[j])>=0.05){     //checks 5% variation in ANY value
      save=1;
      break;                                                 
    }
  }

  if(save==1){                                                //saves in log_matrix and replaces old values
    time_log_matrix[i_log_matrix]=time;
    Serial.print(time);
    Serial.print("\t");
    for(j=0;j<5;j++){
      log_matrix[i_log_matrix][j]=vector[j];
      vector_old[j]=vector[j];
      Serial.print(int(vector[j]));
      Serial.print("\t");
    }
  i_log_matrix++;
  if(i_log_matrix==n_log_matrix){
    i_log_matrix=0;                        //rewinds buffer position
  }
  Serial.print("\n");
  }
}

void setup() {
  /*  MUX ADRESS PINS*/
  pinMode(serial0, OUTPUT);
  pinMode(serial1, OUTPUT);
  pinMode(serial2, OUTPUT);
  
  pin_sel(sondas[0]);                                                //selects first probe
  f_save=0;                                                          //flag to save values in matrix used for control

  /* Pin to read probes, MUX OUT */
  pinMode(input, INPUT);

  //Luminosity sensor
  pinMode(A0, INPUT);

  //water count
  pinMode(baloico, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(baloico), Count_ISR, FALLING); //interrupt

  //Pin button
  pinMode(botao, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(botao ), button_ISR, FALLING); //interrupt

  //Pin to valve control
  pinMode(uln[0],OUTPUT);
  digitalWrite(uln[0], LOW);

  //TIMER1 SETUP
  TCNT1  = 0;                     //reset
  TCCR1A = 0;                     //clear register
  TCCR1B = 0;                     //clear register
  OCR1A = 57156;                  // compare match register 62499->4s  (61457 - IE; 57156 - CTR ) to ensure 4 real sec.
  TCCR1B |= (1 << WGM12);         // CTC mode
  TCCR1B |= (1<<CS10);            //PS=1024
  TCCR1B &= ~(1<<CS11);           //PS=1024
  TCCR1B |= (1<<CS12);            //PS=1024
  TIMSK1 |= (1 << OCIE1A);        // enable timer compare interrupt
  TIMSK1 &= ~(1 << ICIE1);        // disable IC interrupt


  //TIMER2 SETUP timer to 'watchdog' the mux
  TCCR2A = 0;                       // Normal mode
  TCCR2B = 0;
  TCCR2B |= (1 << CS22);
  TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS20);           //PS=1024
  TIMSK2 |= (1 << TOIE2);          // enable overflow interrupt to detect missing input pulses

  TIMSK1 |= (1 << OCIE1A);          // enable timer compare 

  Serial.begin(9600);               //Serial connection

  //////////////USER INTERFACE////////////
   // Adds newline to every command
  cmdMessenger.printLfCr();   

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();

  integral=0;               //resets integral term for PD control
  i_matrix=-1;              //resets matrix line
  
  Serial.println("Setup Complete");				//setup print
}

void loop() {
     // Process incoming serial data, and perform callbacks
  cmdMessenger.feedinSerialData();
  
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

  //If there is watering to be done
  if (flag_rega != 0){
      rega(flag_rega);      //water
      flag_rega = 0;
  }
      
  if(f_t1==1){              //data aquisition routine
    f_t1=0;                 //reset flag
    t2_reset();             //reset timeout timer
    flag_wdt = 0;           //reset timeout timer flag
    aquire();               //aquisition routine
    contas();               //caliblation computations
    save_log_vector();      //saves log 
    
    if(f_save==1){          //save to control buffer
      save_vector();        //save to control buffer
      f_save=0;             //clear flag
    }

    if(i_matrix==n_matrix-1){     //control buffer full
      control();             //call control routine     
    }

  }
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt 
}
 
