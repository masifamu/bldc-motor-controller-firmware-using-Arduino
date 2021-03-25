#include <avr/io.h>
#include <avr/interrupt.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };


#define processorMHz              16
#define PWMFreq                   16666
#define dutyCycleMax              0.5
#define minBattThreVolt           36
#define maxBattThreVolt           56
#define waitAftLowVoltDet         5
#define wheelDia                  0.65
#define HSCutsInOneCycle          266.0


void setHallSensorInterrupts();
void startTimerForThrotle();
void pciSetup(byte);
void beginMotion();
void setPortDirection();
void initializeCPU();
uint16_t readAnalogVoltage(uint8_t);
uint16_t ReadADC(uint8_t);
void InitADC();
void blinkLED(uint8_t);
void blinkLED_50(uint8_t);
void startPWM();
void stopPWM();
void stopMotor();


int timeLapsed=0,num,maxNum=0,period;
static int throtle;
uint16_t batteryVoltage=0;
volatile uint8_t a=0;
bool breakPressed=false;
unsigned int noOfHSCuts=0,minVoltDetTimer=0;;
float motorSpeed=0,battCap=0;

void setup(){
  //place a routine for LED blinking to show the startup
  Serial.begin(9600);
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F(" Punchline"));
  display.println(F("  Energy"));
  display.println(F(" Pvt. Ltd."));
  display.display();
  delay(1000);

  
  blinkLED(2);
  InitADC();
  batteryVoltage=uint16_t(readAnalogVoltage(2)*(5.0/1024.0)*11.0);//channel2 is selected
  while(!(batteryVoltage >= minBattThreVolt && batteryVoltage <= maxBattThreVolt)) blinkLED(5);//if voltage is not correct LED will keep on blinking
  blinkLED(1);//if voltage is correct then upto here only 3 blinks
  
  setPortDirection();
  setHallSensorInterrupts();
  initializeCPU();
  
  period=(processorMHz*1e6*1.0)/(PWMFreq*1.0);
  maxNum=period*dutyCycleMax;

  displayText("Ready"," ");
  do{
    throtle=readAnalogVoltage(0);
    num=(throtle>178)?map(throtle,178,900,0,period*dutyCycleMax):1;
  }while(num<=10);
  
  startTimerForThrotle();
}
void loop(){
    beginMotion();//find the rotor position and start motor for the first time
    do{//now the motor will run continously
        
        //controlling the throtle for changing the PWM width
        throtle=readAnalogVoltage(0);
        num=(throtle>178)?map(throtle,178,900,0,maxNum):1;
        
        
        //Reading the battery voltage for monitoring the health of battery//stop everything in case of low voltage
        batteryVoltage=uint16_t(readAnalogVoltage(2)*(5.0/1024.0)*11);
        if(!(batteryVoltage>=minBattThreVolt && batteryVoltage<=maxBattThreVolt)){
          if(minVoltDetTimer==waitAftLowVoltDet){displayText("Batt volt","Problem"); stopMotor();}
        }else minVoltDetTimer=0;
        //now calculating the remaining capacity of the battery
        battCap=(float)(batteryVoltage-minBattThreVolt)/(maxBattThreVolt-minBattThreVolt)*100.0;

        
        //cutting the power to the motor by managing break here
        if(digitalRead(A1)==0){stopPWM(); while(digitalRead(A1)!= 1) blinkLED_50(2); startPWM();}


        
        
        //monitor the current drawn by the motor




        //speed, distance and time calculation
        static float distance=0;
        static unsigned int OnTime=0;
        if(timeLapsed >= PWMFreq){//becomes true at every one sencond interval
          motorSpeed=(noOfHSCuts/HSCutsInOneCycle)*3.14*wheelDia*3.6;
          distance=distance+motorSpeed/3600;
          OnTime++;
          minVoltDetTimer++;
          noOfHSCuts=0;
          timeLapsed=0;
        }
        
        //sending data to the OLED display
        testdrawstyles(motorSpeed,distance,OnTime,battCap);
        
    }while(1);
}
/*.....................................................................................................*/
//calls every 200us
ISR(TIMER1_OVF_vect){
  OCR1A=num;
  timeLapsed++;
}//116us

/*.....................................................................................................*/
void blinkLED(uint8_t blinkCount){
  DDRB |= 0b00100000;
  for(uint8_t i=0;i<blinkCount;i++){
    PORTB |= 0b00100000;
    _delay_ms(250);
    PORTB &= ~0b00100000;
    _delay_ms(250);
  }
}
/*.....................................................................................................*/
void blinkLED_50(uint8_t blinkCount){
  DDRB |= 0b00100000;
  for(uint8_t i=0;i<blinkCount;i++){
    PORTB |= 0b00100000;
    _delay_ms(30);
    PORTB &= ~0b00100000;
    _delay_ms(30);
  }
}
/*.....................................................................................................*/
void testdrawstyles(float ms,float dis,unsigned int t,float bc) {
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.setTextColor(WHITE); 
  display.setTextSize(2); 
  //display.print(int(rpm1));display.println(F(" RPM"));//max current
  //display.print(int(c));display.println(F(" Cuts "));//max current
  display.print((t/60)/60);display.print(F("h"));//hours
  display.print((t/60)%60);display.print(F("m"));//minutes
  display.print(t%60);display.println(F("s"));//seconds
  display.print(ms);display.println(F("km/h"));//motor speed
  display.print(dis,3);display.println(F("km"));//distance travelled
  display.print(int(bc));display.print(F("% "));//battery capacity
  display.display();
}

void displayText(String string1, String string2){
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner

  display.setTextColor(WHITE); 
  display.setTextSize(2); 
  display.println(string1);//total power
  display.println(string2);//total power
  display.display();
}
/*.....................................................................................................*/
void stopPWM(){
  TCCR1A = 0b00100010; //PWM is diverted
}//2us
/*.....................................................................................................*/
void startPWM(){
  TCCR1A = 0b10000010; //PWM is diverted
}//2us
/*.....................................................................................................*/
void stopMotor(){
  stopPWM();
  //cli();
  while(1) blinkLED(100);
}

/*.....................................................................................................*/
void setHallSensorInterrupts(){
  pciSetup(12);//blue
  pciSetup(11);//green
  pciSetup(8);//yellow
}
/*.....................................................................................................*/
//hall sensor is only used to find the next commutation
void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}
/*.....................................................................................................*/
ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
 {
  static int count;
      a=PINB & 0b00011001;//0b 0,0,0,Blue,Green,0,0,Yellow //0b00011001 is used to filter out blue,green,yellow bits.
     if(count != a){
       if(a==16){//CH_AL
          PORTD &=0b01001111;//0bA,B,C,Al,Bl,Cl,Tx,Rx
          PORTD |=0b01001100;//0bA,B,C,Al,Bl,Cl,Tx,Rx 
          noOfHSCuts++;
        }else if(a==24){//BH_AL
          PORTD &=0b10001111;//0bA,B,C,Al,Bl,Cl,Tx,Rx
          PORTD |=0b10001100;//0bA,B,C,Al,Bl,Cl,Tx,Rx
          noOfHSCuts++;
        }else if(a==8){//BH_CL
          PORTD &=0b10011011;//0bA,B,C,Al,Bl,Cl,Tx,Rx
          PORTD |=0b10011000;//0bA,B,C,Al,Bl,Cl,Tx,Rx
          noOfHSCuts++;
        }else if(a==9){//AH_CL
          PORTD &=0b00011011;//0bA,B,C,Al,Bl,Cl,Tx,Rx
          PORTD |=0b00011000;//0bA,B,C,Al,Bl,Cl,Tx,Rx
          noOfHSCuts++;
        }else if(a==1){//AH_BL
          PORTD &=0b00010111;//0bA,B,C,Al,Bl,Cl,Tx,Rx     reset every pin without affecting Tx,Rx,Bl.
          PORTD |=0b00010100;//0bA,B,C,Al,Bl,Cl,Tx,Rx     set only Bl without affecting the other pins.
          noOfHSCuts++;
        }else if(a==17){//CH_BL
          PORTD &=0b01010111;//0bA,B,C,Al,Bl,Cl,Tx,Rx
          PORTD |=0b01010100;//0bA,B,C,Al,Bl,Cl,Tx,Rx
          noOfHSCuts++;
        }
      }
      count=a;
 }//20us
/*.....................................................................................................*/
void beginMotion(){
   a=PINB & 0b00011001;//0b00011001 is used to filter out blue,green,yellow bits.
   if(a==16){//CH_AL
      PORTD &=0b01001111;//0bA,B,C,Al,Bl,Cl,Tx,Rx
      PORTD |=0b01001100;//0bA,B,C,Al,Bl,Cl,Tx,Rx 
    }
    if(a==24){//BH_AL
      PORTD &=0b10001111;//0bA,B,C,Al,Bl,Cl,Tx,Rx
      PORTD |=0b10001100;//0bA,B,C,Al,Bl,Cl,Tx,Rx
      
    }
    if(a==8){//BH_CL
       PORTD &=0b10011011;//0bA,B,C,Al,Bl,Cl,Tx,Rx
       PORTD |=0b10011000;//0bA,B,C,Al,Bl,Cl,Tx,Rx
    }
    if(a==9){//AH_CL
      PORTD &=0b00011011;//0bA,B,C,Al,Bl,Cl,Tx,Rx
      PORTD |=0b00011000;//0bA,B,C,Al,Bl,Cl,Tx,Rx
    }
    if(a==1){//AH_BL
      PORTD &=0b00010111;//0bA,B,C,Al,Bl,Cl,Tx,Rx     reset every pin without affecting Tx,Rx,Bl.
      PORTD |=0b00010100;//0bA,B,C,Al,Bl,Cl,Tx,Rx     set only Bl without affecting the other pins.
    }
    if(a==17){//CH_BL
      PORTD &=0b01010111;//0bA,B,C,Al,Bl,Cl,Tx,Rx
      PORTD |=0b01010100;//0bA,B,C,Al,Bl,Cl,Tx,Rx
    }
}






/*.....................................................................................................*/
void setPortDirection(){
  DDRD |= 0b11111100;   //0bA,B,C,Al,Bl,Cl 
  DDRB = 0b11100110;   //XTAL2,XTAL1,D13,Blue,Green,PWN10,PWM9,Yellow
  //PORTB |= 0b00011001;  //making Blue, green, yellow pin high so that the Hi-Z imedance state can be avoided//because we are using external pullUp resistor
  
  pinMode(A0,INPUT);//throtle
  pinMode(A1,INPUT_PULLUP);//break
  pinMode(A2,INPUT);// batteryVoltage
  pinMode(A3,INPUT);//
  pinMode(A4,INPUT);//f/r
}
/*.....................................................................................................*/
void initializeCPU(){
  num=0;
  batteryVoltage=0;
}
/*.....................................................................................................*/
void startTimerForThrotle(){
  // Clear Timer/Counter Control Registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = 0b10000010;
  TCCR1B = 0b00011001;
  TIMSK1 = 0b00000001;
  ICR1=period;//starting period of trangular wave
  OCR1A=1;
}






/*.....................................................................................................*/
uint16_t readAnalogVoltage(uint8_t adcChannel){
  uint16_t value=0;
  for(int i=0;i<10;i++){
    if(i>=2) value=value+ReadADC(adcChannel); else ReadADC(adcChannel); //leaving the first two samples for avoiding any undesired value
  }
  return(uint16_t((value*1.0)/8.0));
}
/*.....................................................................................................*/
void InitADC()
{
 // Select Vref=AVcc
 ADMUX |= (1<<REFS0);
 //set prescaller to 128 and enable ADC 
 ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);    
}
/*.....................................................................................................*/
uint16_t ReadADC(uint8_t ADCchannel)
{
 //select ADC channel with safety mask
 ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
 //single conversion mode
 ADCSRA |= (1<<ADSC);
 // wait until ADC conversion is complete
 while( ADCSRA & (1<<ADSC) );
 return ADC;
}
/*.....................................................................................................*/
