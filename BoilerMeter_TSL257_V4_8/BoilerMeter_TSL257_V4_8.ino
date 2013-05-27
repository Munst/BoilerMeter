 /*
  ,---.   .---.  .-.  .-.,---.  ,---.            ,---. _______ ,---.  ,---.  .,-.,---.   
  | .-.\ / .-. ) | |/\| || .-'  | .-.\  |\    /| | .-'|__   __|| .-'  | .-.\  |(|| .-'   
  | |-' )| | |(_)| /  \ || `-.  | `-'/  |(\  / | | `-.  )| |   | `-.  | `-'/  (_)| `-.   
  | |--' | | | | |  /\  || .-'  |   (   (_)\/  | | .-' (_) |   | .-'  |   (   | || .-'   
  | |    \ `-' / |(/  \ ||  `--.| |\ \  | \  / | |  `--. | |   |  `--.| |\ \  | ||  `--. 
  /(      )---'  (_)   \|/( __.'|_| \)\ | |\/| | /( __.' `-'   /( __.'|_| \)\ `-'/( __.' 
 (__)    (_)            (__)        (__)'-'  '-'(__)          (__)        (__)  (__)     
 //##################################################################################################
 // BoilerMeter measurement - keeps a running count of the hours of operation, using the TSL257 
 // to detect the flame through the inspection window. Also measures temperature to see if we can get 
 // get some useful data for the Riello Rl/28 boiler.
 // Eamonn O'Brien Feb 2013  :  O'Brien Energy Solutions
 // Based on JeeLabs RF12 library http://jeelabs.org/2009/02/10/rfm12b-library-for-arduino/
 // Based on original work by Glyn Hudson, Trystan Lea
 // 19th Feb 2013: V1 - initial version. 
 // 20th Feb 2013: V4_2 - implement low power ideas from http://openenergymonitor.org/emon/buildingblocks/pulse-counting-sleep
 //                and put in code to sleep RF when Boiler is off.
 // 5th  Apr 2013: V4_5 - initialised StopWatch to be =0 instead of digitalread(TSL257_pin); removed delay(10000) in loop 'if (StopWatch=1)' statement; 
 //                increased debounce interval from 200 to 500 to see if this improves turnoff precision; 
 // 9th  Apr 2013: V_4_6 - used Iinterrupt()/noInterrupts() wrappers; adding watchdog setup causes continual reset. Also removed  
 //                "ISR(WDT_vect) { Sleepy::watchdogEvent(); }" since it's not used.
 // 16th Apr 2013: V_4_7 - removed LED blink on sleep and reduced the LED on delay in loop to improve battery life.
 // GNU GPL V3
 //##################################################################################################
 */
 //JeeLabs libraries 
 #include <Ports.h>
 #include <RF12.h>
 //#include <avr/eeprom.h>
 #include <avr/sleep.h>
 #include <avr/power.h> 
 //ISR(WDT_vect) { Sleepy::watchdogEvent(); } 
 #include <avr/wdt.h>
 #include <util/crc16.h>  
 #include <OneWire.h>
 #include <JeeLib.h>                                                     // Download JeeLib: http://github.com/jcw/jeelib
 #include <DallasTemperature.h>

 //##################################################################################################
 // Serial print settings - disable all serial prints if SERIAL 0 - increases long term stability 
 //##################################################################################################
 #define SERIAL 0

 //##################################################################################################
 // RF12 settings 
 //##################################################################################################
 #define nodeID 17         //in the range 1-30; complies with wireless node ids standard at http://openenergymonitor.org/emon/node/1483
 #define network 212      //default network group (can be in the range 1-250). All nodes required to communicate together must be on the same network group
 #define freq RF12_868MHZ     //Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.

 // set the sync mode to 2 if the fuses are still the Arduino default
 // mode 3 (full powerdown) can only be used with 258 CK startup fuses
 #define RADIO_SYNC_MODE 2
 #define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

 //##################################################################################################
 // LED Indicator  
 //##################################################################################################
 # define LEDpin 9          //hardwired on emonTx PCB
   unsigned long starttime = millis();
   unsigned long duration;
 //##################################################################################################
 //Data Structure
 //################################################################################################## 
 typedef struct {int battery, runtime, temp0, temp1;} Payload2;
 Payload2 BoilerMeter;
 int temp0, temp1;
 //##################################################################################################
 // Temperature  
 //##################################################################################################
 
 #define ONE_WIRE_BUS 4
 #define TEMPERATURE_PRECISION 9                   // Min resolution, since I'm running off AA batteries
 OneWire oneWire(ONE_WIRE_BUS);                    // Setup a oneWire instance 
 DallasTemperature sensors(&oneWire);              // Pass our oneWire reference to Dallas Temperature.
 int numberOfDevices;
 DeviceAddress sensor;
 const int sensorResolution = 9;                                        //DS18B20 resolution 9,10,11 or 12bit corresponding to (0.5, 0.25, 0.125, 0.0625 degrees C LSB), lower resolution means lower powe

 //##################################################################################################
 // TSL257 settings 
 //##################################################################################################
  # define TSL257_pin 3          
  int runtime = 0;
  float elapsed,PreviousMillis,m,hrs,Fract_hrs,BoilerOff=0;
  volatile int StopWatch = 0;
  volatile int count =0;
  volatile static unsigned long last_interrupt_time = 0;
  unsigned long TotalTime,BoilerOn = 0;
 //##################################################################################################
 // Sleep  
 //##################################################################################################
  void enterSleep(void)
  {
  delay(100); 
  /*digitalWrite(LEDpin, HIGH);    //flash LED - very quickly 
  delay(2); 
  digitalWrite(LEDpin, LOW); */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  sleep_disable(); 
  }
 //##################################################################################################
 //SETUP
 //##################################################################################################
 void setup()
 {  
    if (!SERIAL) {
    Serial.println("serial disabled"); 
    Serial.end();
  }  
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin, HIGH);    //turn on LED 

  Serial.begin(9600);
  Serial.println("BoilerMeter V4_2");
  Serial.println("powermeter.ie");
  Serial.print("Node: ");     Serial.print(nodeID); 
  Serial.print(" Freq: ");    Serial.print(freq);
  Serial.print(" Network: "); Serial.println(network);
  

  sensors.begin();   //Start the library
  if (!sensors.getAddress(sensor, 0)) Serial.println("Unable to find temperarture sensor");
  sensors.setResolution(sensor, sensorResolution);
  Serial.print("Temperature Sensor Resolution:");
  Serial.println(sensors.getResolution(sensor), DEC); 
  
  rf12_initialize(nodeID,freq,network); 
  rf12_control(0xC040);                                                 // set low-battery level to 2.2V i.s.o. 3.1V
  delay(10);
  rf12_sleep(RF12_SLEEP);
  attachInterrupt(1, onPulse, CHANGE); 
  //wdt_enable(WDTO_8S);
  Serial.println("Initialisation complete.");
  digitalWrite(LEDpin, LOW);    //turn off LED 
 }
 //##################################################################################################
 //LOOP
 //##################################################################################################
 void loop()
 {
      noInterrupts();
      duration = millis()-starttime;   
      sensors.requestTemperatures();            //sensors - give me your temps....
      float temp0=(sensors.getTempCByIndex(0));
      float temp1=(sensors.getTempCByIndex(1));
      BoilerMeter.battery=readVcc();
      BoilerMeter.temp0 = (temp0*100);
      BoilerMeter.temp1 = (temp1*100);
      
  if (duration >= 60000) 
    {
     send_rf_data() ;
     starttime = millis();
     //Serial.println("RF is on");
    }  
  
 //##################################################################################################
 // Boiler runtime 
 //##################################################################################################

  if ( StopWatch == 1 ) {
           digitalWrite(LEDpin, HIGH);    //flash LED - very quickly just to show us we're alive 
           delay(2); 
           digitalWrite(LEDpin, LOW); 
           elapsed = millis() - PreviousMillis;  //measure time between up and down
           //Serial.print("elapsed (loop)= "); Serial.println(elapsed);
           TotalTime += elapsed;
           //Serial.print("Total(BoilerON)= "); Serial.println(TotalTime);
            m = (TotalTime/60000);
            BoilerMeter.runtime=int((m/60)*100);
            // Serial.print("elapsed(BoilerOFF) = "); Serial.println(elapsed);
            // Serial.print("m = "); Serial.println(m);
            // Serial.print("runtime = "); Serial.println(runtime,DEC);
            PreviousMillis = millis();
            //delay(10000);
         } else if ( StopWatch == 0 ){ 
                 PreviousMillis = millis();
                 interrupts();
                 //Serial.print("Total(BoilerOFF)= "); Serial.println(TotalTime);
                 //Serial.print("Stopwatch is = "); Serial.println(StopWatch);
                 //Serial.print("elapsed(BoilerOFF) = "); Serial.println(elapsed); 
                 enterSleep();           
         }

 //##################################################################################################
 // Send data via RF 
 //##################################################################################################
    
  if (SERIAL) { serial_output();}
  
  digitalWrite(LEDpin, HIGH);    //flash LED - very quickly 
  delay(1000); 
  digitalWrite(LEDpin, LOW); 
wdt_reset();  
 }
 //##################################################################################################
 // The interrupt routine - runs each time a falling edge of a pulse is detected
 //##################################################################################################

void send_rf_data()
{
  rf12_sleep(RF12_WAKEUP);
  // if ready to send + exit loop if it gets stuck as it seems too
  int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
  rf12_sendStart(0, &BoilerMeter, sizeof BoilerMeter);
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
  rf12_sendWait(2);
  rf12_sleep(RF12_SLEEP);
}

 void onPulse()
 {
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if ( interrupt_time - last_interrupt_time > 500 && digitalRead(TSL257_pin) == HIGH){
       StopWatch = 1; 
      } else if ( interrupt_time - last_interrupt_time > 500 && digitalRead(TSL257_pin) == LOW){
       StopWatch = 0;
       count++;
      }
  last_interrupt_time = interrupt_time;
 }

 //##################################################################################################
 // Read current emonTx battery voltage
 //##################################################################################################
 long readVcc() {
  long result;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result;
  return result;
 }

 //##################################################################################################
 // Serial output
 //##################################################################################################
 void serial_output()
 {
    Serial.print("Boiler has turned OFF "); Serial.print(count); Serial.println(" Times");
    Serial.print(" |Battery : ");                Serial.println(BoilerMeter.battery);    
    Serial.print(" |Boiler runtime (hrs*100) //divide by 100 in emoncms) : ");      Serial.println(BoilerMeter.runtime); 
    Serial.print(" |Temp1 : ");                  Serial.println(BoilerMeter.temp0);
    Serial.print(" |Temp2 : ");                  Serial.println(BoilerMeter.temp1);
 }
