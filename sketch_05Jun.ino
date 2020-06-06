    //Date :- 09-05-19
    
    #include "Nextion.h"
    #include "Seeed_BME280.h"
    #include <math.h> 
    #include <Wire.h> 
    BME280 bme280;
    #include <TimerOne.h>
    #include <TimerThree.h>
    /*
     *******************************************************************
       Nextion components
     *******************************************************************
    */
    NexPicture p0 = NexPicture(1, 9, "p0");// Motor Speed
    NexPicture p1 = NexPicture(1, 10, "p1");// I/P Pressure
    NexPicture p2 = NexPicture(1, 11, "p2");// O/P Pressure
    NexPicture p3 = NexPicture(1, 12, "p3");// Valume
    NexButton b4 = NexButton(1,5,"b4"); // Start button
    NexButton b5 = NexButton(1,6,"b5"); // Stop button
    NexNumber n0  = NexNumber(1,1,"n0"); // Motor Speed value
    NexNumber n1  = NexNumber(1,2,"n1"); // I/P Pressure value
    NexNumber n2  = NexNumber(1,3,"n2"); // O/P Pressure value
    NexNumber n3  = NexNumber(1,4,"n3"); // Valume threshold value
    NexNumber n4  = NexNumber(1,7,"n4");   // Valume current value
    NexNumber n5  = NexNumber(1,15,"n5");
    NexNumber n6  = NexNumber(1,16,"n6");
    
    NexText tk0  = NexText(4,13,"tk0");     
    NexText tk1  = NexText(4,15,"tk1");
    NexPicture pk0 = NexPicture(4,14,"pk0");
    NexPicture pks = NexPicture(4,17,"pks"); 
    NexNumber nk0  = NexNumber(4,16,"nk0");
    
    NexText tk10  = NexText(5,13,"tk10");     
    NexText tk11  = NexText(5,14,"tk11");
    NexPicture pk1 = NexPicture(5,16,"pk1");
    NexPicture pks1 = NexPicture(5,17,"pks1");
    NexNumber nk1  = NexNumber(5,15,"nk1");
    
    NexText tk20  = NexText(6,13,"tk20");     
    NexText tk21  = NexText(6,15,"tk21");
    NexPicture pk2 = NexPicture(6,14,"pk2");
    NexPicture pks2 = NexPicture(6,17,"pks2");
    NexNumber nk2  = NexNumber(6,16,"nk2");
    
    //
    
    const int ena = 2;//enable pin to port pin 2 
    const int dir = 3; //direction pin to port pin 3
    const int pul = 4; //pulse pin to port pin 4
    int pulseState  = 0;
    float rpm = 50;//initialise rpm value
    uint32_t number = 50;
    long rpmks =50; // flag to set rpm with keypad
    long rpmks1;
    long rpmks2;
    long knum;
    int rpm_key1 = 0;
    char buffer1[10] = {0};
    int b_off=0;//initialise button 
    volatile int m_start=0;//initialise for press start button 
    volatile int m_stop=1;//initialise for press stop button
    int pulse200reset = 0; //used for every 200 steps to read the pressureData_i  
    long offtimedlay = 6000;//initialise offtimedelay
    long offtimedlay1 = 6000;//initialise offtimedelay
    long pressureRaw_i = 0;//intialise pressure
    long pressureData_i;
    float va_p1=0;
    int p_mmHg=0;
    int p_mmHg_display=0;
    
    int back_state = 0;
    int setpending = 1;
    int set_ms_temp = 0; // to set the motor speed directly from the home page
    int set_ms_temp1 = 0;
    int set_home =0;  // home button is used for set
    int set_rpm =0;
    int rpm_key =0;
    int rpm_sld =0;
    int pin_key =0;
    int p_inks =50;
    int p_out_key =0;
    long p_outks =0;
    uint32_t buffer = 0;
    uint32_t buffer2 = 0;
    uint32_t buffer3 = 0;
    int sensorValue_p;
    int sensorValue_n;
    int count =0;
    int change_rpm =1;
    int new_rpm;
    int new_rpm1;
    uint8_t low, high;
    
    NexTouch *nex_listen_list[] = 
    {
        &p0,
        &b4,
        &b5,
        &pk0,
        &pk1,
        &pk2,  
        &pks,
        &pks1,
        &pks2,
        NULL
    };
     /*
     *******************************************************************
       Nextion Callback Functions page 4(keypad)
     *******************************************************************
    */
    
    void pksPopCallback(void *ptr)//function call for keypad set  button
          { 
             dbSerialPrintln("pksPopCallback");//pop the button bks and print on serial monitor 
              rpm_key = 1; 
              if(rpm_key ==1)
             { 
             rpmks = buffer;
             n0.setValue(rpmks);
             rpm = rpmks;
             n2.setValue(p_outks);
             n5.setValue(p_inks); 
             //dbSerialPrintln(buffer);
             }         
      }
      void pks1PopCallback(void *ptr)//function call for keypad set  button
          { 
             dbSerialPrintln("pks1PopCallback");//pop the button bks and print on serial monitor 
              pin_key =1;
             if(pin_key ==1)      
         { 
             p_inks = buffer2;
             n5.setValue(p_inks);  
             n2.setValue(p_outks);
             n0.setValue(rpmks);
             rpm = rpmks;
            dbSerialPrintln(p_inks); 
            dbSerialPrintln(p_outks);
         }
          }
      void pks2PopCallback(void *ptr)//function call for keypad set  button
          { 
            dbSerialPrintln("pks2PopCallback");//pop the button bks and print on serial monitor 
              p_out_key =1;
             if(p_out_key ==1)     
           { 
             p_outks = buffer3;
             n2.setValue(p_outks);
             n5.setValue(p_inks); 
             n0.setValue(rpmks);
             rpm = rpmks;
             dbSerialPrintln(p_inks); 
             dbSerialPrintln(p_outks); 
          }
          } 
          void pk0PopCallback(void *ptr)//function call for  button 
          { 
          dbSerialPrintln("pk0PopCallback");
             nk0.getValue(&buffer);//set the slider value to textbox t10
             dbSerialPrintln(buffer);
            }    
          void pk1PopCallback(void *ptr)//function call for  button  
      {
            dbSerialPrintln("pk1PopCallback");     
             nk1.getValue(&buffer2);//set the slider value to textbox t10
             dbSerialPrintln(buffer2);  
      }
          void pk2PopCallback(void *ptr)//function call for  button 
             
      {
             dbSerialPrintln("pk1PopCallback"); 
             nk2.getValue(&buffer3);//set the slider value to textbox t10
             dbSerialPrintln(buffer3);         
      }
       /*
     *******************************************************************
       Nextion Callback Functions page 1
     *******************************************************************
    */
    void p0PopCallback(void *ptr)//function call for pop the button to set value for n10
          { 
            dbSerialPrintln("p0PopCallback");//poping the button b10 and print on serial monitor
            set_ms_temp =1; //to set the motor speed directly from the home page
            set_ms_temp1 =0; //to set the motor speed directly from the home page
            if(set_ms_temp == 1)
            { 
             count =0;
             dbSerialPrintln(count);
            }
          } 
    void b4PopCallback(void *ptr)  // function call for Start button
          { 
            dbSerialPrintln("b4PopCallback");//pop the button b4 and print on serial monitor
            m_start = 1;  //button pressed
            m_stop =  0;  //button not pressed
            set_ms_temp =0;
            if(set_ms_temp == 0)
            { 
             count =0;
             dbSerialPrintln("count : ");
             dbSerialPrintln(count);
            }
            n0.setValue(rpm);//set rpm to button n0
            n3.setValue(30200);
            n5.setValue(p_inks); 
           digitalWrite(ena,HIGH);//set motor enable is high
            offtimedlay = ((300000/rpm));//calculate offtimedelay
         }
    void b5PopCallback(void *ptr)//function call for Stop button
          { 
            dbSerialPrintln("b5PopCallback");//pop the button b5 and print on serial monitor
             m_start = 0;//button pressed
             m_stop=1;  //button not pressed
            n0.setValue(0);//set the numberbox n0 to zero
            n5.setValue(p_inks); 
            digitalWrite(ena,LOW);//set motor enable is low
          }
    
    /*
     *******************************************************************
       void setup
     *******************************************************************
    */       
    void setup(void)
    {
      ADMUX = (1<<REFS0) | (1<<MUX3) |(1<<MUX1) | (1<<MUX0) ;                //choose AVCC for reference ADC voltage
      ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);    //enable ADC, ADC frequency=16MB/128=125kHz (set of prescaler)
      ADCSRB = (1<<ACME) | (1<<ADTS0);
    
      
      pinMode(ena, OUTPUT);//set enable pin as output
      pinMode(dir, OUTPUT);//set direction pin as output
      pinMode(pul, OUTPUT);//set pulse pin as output
      pinMode(A0, INPUT); 
      digitalWrite(dir, HIGH); //set direction to high
      digitalWrite(pul, HIGH);//set pulse to high
      if(!bme280.init())
      {
        Serial.println("Device error!"); //print error message
      }
        nexInit();
        Serial.begin(115200);//set baud rate
    
        p0.attachPop(p0PopCallback);//attach callback function of b0 pop touch event
        b4.attachPop(b4PopCallback);//attach callback function of b4 pop touch event
        b5.attachPop(b5PopCallback);//attach callback function of b5 pop touch event
        pks.attachPop(pksPopCallback);//attach callback function of b13 pop touch event
        pk0.attachPop(pk0PopCallback);//attach callback function of b13 pop touch event
        pk1.attachPop(pk1PopCallback);//attach callback function of b13 pop touch event
        pk2.attachPop(pk2PopCallback);//attach callback function of b13 pop touch event
        pks.attachPop(pksPopCallback);
        pks1.attachPop(pks1PopCallback);//attach callback function of b13 pop touch event
        pks2.attachPop(pks2PopCallback);//attach callback function of b13 pop touch event
        dbSerialPrintln("setup done");//print setup done on serial monitor 
        Timer1.initialize(1000);//initialise timer1 to 5seconds 
        Timer3.initialize(1000);//initialise timer1 to 5seconds 
      //  Timer1.attachInterrupt(Motor_Step); // Motor 1 Step move
        Timer3.attachInterrupt(Motor_Step); // Motor 1 Step move
        TIMSK1 = 0;  // Prevent further TIMER 1 interrupts   
        Serial.println("Setup  done");    
    } 
    /*
     *******************************************************************
       Motor_Step function
     *******************************************************************
    */  
    void Motor_Step(void)//function call for motor step
    {
      if ((m_start == 1) && (m_stop == 0)) //start button is pressed 
      { 
        digitalWrite(pul,HIGH);//set pulse to high
        delayMicroseconds(50);
         digitalWrite(pul,LOW);//set pulse to low
        delayMicroseconds(50);//pauses for 300 microseconds
        Timer3.initialize(offtimedlay);//  REinitialise timer1 to offtimedelay  , for required rpm
        }
       }
       /*
     *******************************************************************
                 void loop
     *******************************************************************
    */          
     void loop(void)
     
    { 
        pulse200reset++;
      if ((m_start == 1) && (m_stop == 0))
      {
         TIMSK1 |= (1 << TOIE1);//TOIE1 shifted by 1
         interrupts();//enable interrupt
         if ((pulse200reset >20000) && (set_ms_temp==0)&& m_start && !m_stop)
            {
             pressure_displayCallback();
             pulse200reset =0;
            }  
         //pressure regulation, reduce rpm..   
         if(pressureData_i < 95300 && m_start && !m_stop && (set_ms_temp == 0))
           { 
               if( p_mmHg >= p_inks) //if sensor o/p is higher then set value   
        
             {
                 
                  new_rpm = rpm*0.60;   //reduce by 40%
                  Serial.println(new_rpm);
              //    rpm = new_rpm;
                  n0.setValue(new_rpm);//set new rpm to button n0
                  n3.setValue(30200);   //defalt Max value
                  digitalWrite(ena,HIGH);//set motor enable is high
                  offtimedlay1 = ((300000/new_rpm));//calculate offtimedelay
                   Timer1.initialize(offtimedlay1);
              }
       
    //      digitalWrite(ena, HIGH); /// set enabale pin to High// ???vnb
          }
         
        /* int sensorMin = 95100 ;
         int sensorMax = 95300;
       int range = map(pressureData_i, sensorMin,sensorMax,0,1);
       //if(pressureData_i > 95300) 
       switch (range) {
        case 0:  
         digitalWrite(ena, HIGH);
        case 1: */
            while(pressureData_i > 95300)  // pressure is higher then max limirt
            {    
           digitalWrite(ena, LOW); // set enabale pin to ground
           exit;
            }    
         }
             nexLoop(nex_listen_list);
     }
       /*
     *******************************************************************
            pressure_display function   
     *******************************************************************
    */
    void pressure_displayCallback()
    { 
       sensorValue_p = read_differential();
       float va = sensorValue_p * 4.68/1024;  
       float p_psi=  va*10*0.2584;
       p_mmHg = p_psi*51.714;
        Serial.print("va : ");
        Serial.println(va);
        Serial.print("p_psi : ");
        Serial.println(p_psi);
        Serial.print("p_mmHg : ");
        Serial.println(p_mmHg); 
        Serial.print("sensorValue_p : ");
        Serial.println(sensorValue_p);
        Serial.print("pressureData_i ");
        Serial.println(pressureData_i);
      
       pressureRaw_i = bme280.getPressure();//get values of pressure sensor
        char buffer[16] = {0};
        ltoa(pressureRaw_i, buffer, 10);//covert int to string
        pressureData_i = atol(buffer);
        n4.setValue(pressureData_i);
        ltoa(p_mmHg, buffer, 10);//covert int to string
        p_mmHg_display = atol(buffer);
        n1.setValue(p_mmHg_display);   
    }
    
    int16_t read_differential() {
      
      ADCSRA |= (1<<ADSC);            //start conversion
      while (ADCSRA & (1<<ADSC));     //wait untill coversion be completed(ADSC=0);
    
      low = ADCL;
      high = ADCH;
      //sensorValue = high+low; 
      return (high << 8) | low;       //arrange (ADCH ADCL) to a 16 bit and return it's value.
      }
    
    /*void rpm_reduce()
     {
     while(count<change_rpm)
            {
      //  for(i=0;i<=5;i++)
        //{
            new_rpm = rpm*0.6;
            Serial.println(new_rpm);
            n0.setValue(new_rpm);//set rpm to button n0
            n3.setValue(30200);
            digitalWrite(ena,HIGH);//set motor enable is high
            offtimedlay = ((300000/new_rpm));//calculate offtimedelay
            count++;
            }
          
     }*/
