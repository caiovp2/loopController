/* LOOP Controller

Author: Caio Marciano Santos
Date: Dec, 2019
Lastest update: Jan, 2021

useful links:
http://howtomechatronics.com/tutorials/arduino/rotary-encoder-works-use-arduino/
http://www.labbookpages.co.uk/electronics/debounce.html
https://os.mbed.com/questions/6206/UARTs-FRDM-K64F/

*/
 
/*________________________________ LIBRARIES ________________________________*/
 
#include "mbed.h" // Default mbed library
#include "TextLCD.h" // LCD library for driving 4-bit HD44780-based LCDs
#include "EthernetInterface.h" // Ethernet support library (for future use)

/*_______________________________ COMPACT EDFA _______________________________*/


/* COMPACT EDFA (RS232)
Communication mode: Binary
Communication baud rate: 9600 baud
Data bits: 8 bits
Parity check: None
Stop bits: 1

ALARMS:
Bit 0(LSB): pump current is over limit
Bit 1: signal output power is less than 1dBm of target output power. 
Bit 2: signal output power is over than 1dBm of target output power.
Bit 3: pump power is less than 5mW of target output power.
Bit 4: temperature inside module is over 65°C

(Running mode definition)
APC mode :0x00h
Constant pump current mode :0x01h
*/

// EDFA control definitions
#define R_PUMP_CURRENT 0x01 // Read pump current
#define R_OUTPUT_POWER 0x02 // Read output power
#define R_PUMP_POWER 0x03 // Read pump power
#define R_ALARM 0x05 // Read alarm
#define R_TEMPERATURE 0x06 // Read temperature
#define R_RUNNING_MODE 0x07 // Read EDFA running mode
#define S_PUMP_CURRENT 0x08 // Set pump current
#define S_OUTPUT_POWER 0x09 // Set output power
#define S_RUNNING_MODE 0x0B // Set EDFA running mode
#define PUMP_OFF 0x0C // Turn off pump
#define PUMP_ON 0x0D // Turn on pump
#define PROGRAM_RESET 0x0E // Reset EDFA

/*____________________________ PIN CONFIGURATION ____________________________*/

// Serial Definitions
Serial edfa1(D15,D14); // Pre-Amp EDFA - Pins (TX,RX) - uart 4
Serial edfa2(D1,D0); // Booster EDFA - Pins (TX,RX) - uart 3 

// LCD Definition
TextLCD lcd(D2,D3,D4,D5,D6,D7); //Pins (rs,e,d4-d7)

// Digital Pins for Pulse Control
DigitalOut input1(D12); // H-Bridge In1
DigitalOut input3(D11); // H-bridge In3

// Rotary Encoder 
DigitalIn rotary_A(D8); //Rotary Encoder Knob Pin A (CLK)
DigitalIn rotary_B(D9); //Rotary Encoder Pin B (DT)
DigitalIn button(D10); //Rotary Encoder Button (SW)

// LED
DigitalOut led(D13); // Indicator LED

/*________________________________ INTERRUPTS ________________________________*/

Ticker timer; // Time interrupt
InterruptIn Rise(D8); // Rotary Encoder Pin A Pulse Interrupt (Left rotation)
InterruptIn ButtonRise(D10); // Rotary Encoder Button Interrupt

/*________________________________ FUNCTIONS ________________________________*/

void A_rise(); // Rotary Encoder Pin A Interrupt Function
void button_rise(); // Rotary Encoder Button Interrupt Function
void debounce(); // Debounce function for the rotary encoder
void display(int page, bool blink); // LCD Display auxiliar function
void print_alarm(int edfa); // Function to print EDFA Alarms
void set_edfa(uint8_t command, int edfa, uint16_t data=0); // EDFA set function
uint16_t read_edfa(uint8_t command, int edfa); // EDFA read function

/*________________________________ VARIABLES ________________________________*/

int fiber_spans = 2;
int time_in_fiber = fiber_spans*245; // Time of the pulse in the 50km fiber span (us)
int time_ofdm_packet = 200; // Time of the OFDM data (us)
int loop_number=1;
int pulse_timer =0;
bool send_pulse = false;
bool keep_dc = false;
// Variable Definition
int page = 1;
int pressed_time = 0;
bool up = false; // Check for transition of the wave
bool press = false; // Check for transition of the wave
uint8_t deb=0xFF; // Debounce Variable

int set_stage = 0;
bool set_line = true;

int pump_value=0;
int out_value=0;

//______________________________________________________________________________

    int main() {
        // Start interrupts
        Rise.rise(&A_rise);
        ButtonRise.rise(&button_rise);
        timer.attach(&debounce, 0.001);
        
        // Set format for RS232
        edfa1.baud(9600);
        edfa1.format(8,SerialBase::None,1);
        edfa2.baud(9600);
        edfa2.format(8,SerialBase::None,1);
        
        //set_edfa(S_PUMP_CURRENT,1000); //3000 = maximum
        set_edfa(S_PUMP_CURRENT,1,0);
        set_edfa(PUMP_OFF,1);
        set_edfa(S_PUMP_CURRENT,2,0);
        set_edfa(PUMP_OFF,2);
        
        lcd.cls();
        lcd.printf("Setting EDFAs");
        wait_ms(5000);
        
        // Keep DC on Trigger
        input3=1;
        
        // Main Loop
        while(1){ 
        
            // If send_pulse was activated, send the pulse
            
            if(pulse_timer>=1000){
                if(send_pulse){
                    input1=1;
                    wait_us(time_in_fiber - time_ofdm_packet); // 200us = 0,2ms
                    input3=0;
                    wait_us(time_ofdm_packet); // 200us = 0,2ms
                }
                if(keep_dc){
                    input1=0;
                    wait_us((loop_number-1)*time_in_fiber + time_in_fiber - time_ofdm_packet);
                    input3=1;
                }
                pulse_timer=0;
            }
               
             
            if((set_stage==1 || set_stage==2) && (page==1 || page==3)){
                display(page,true);
            }else if(set_stage==3 && (page == 1 || page ==3)){
                lcd.cls();
                lcd.printf("Setting...");
                if (page==1){
                    set_edfa(S_PUMP_CURRENT,1,pump_value*10);
                    if(pump_value!=0){
                        set_edfa(PUMP_ON,1);
                    }else{
                        set_edfa(PUMP_OFF,1);
                    }
                }else{ //page 3
                    set_edfa(S_PUMP_CURRENT,2,pump_value*10);
                    if(pump_value!=0){
                        set_edfa(PUMP_ON,2);
                    }else{
                        set_edfa(PUMP_OFF,2);
                    }
                }
                set_stage = 0;          
            }else {    
                display(page,false);
            }    
          
        }  
    }
//______________________________________________________________________________

void set_edfa(uint8_t command,int edfa, uint16_t data){
    if (command == PUMP_ON || command == PUMP_OFF || command == PROGRAM_RESET){
        switch(edfa){
            case 1:    
            edfa1.putc(command); // Set command
            break;
            
            case 2:
            edfa2.putc(command);
            break;
        }
        
        if(command == PUMP_ON) led=1;
        if(command == PUMP_OFF) led=0;
    }
    else if (command == S_PUMP_CURRENT){
        char MSB,LSB;
        LSB = data & 0x00FF;
        MSB = data >> 8;
        switch(edfa){
            case 1:
            edfa1.putc(S_RUNNING_MODE); // Set Running Mode
            edfa1.putc(0x01); // Constant Current Mode
            edfa1.putc(command);
            edfa1.putc(MSB);
            edfa1.putc(LSB);
            break;
            
            case 2:
            edfa2.putc(S_RUNNING_MODE); // Set Running Mode
            edfa2.putc(0x01); // Constant Current Mode
            edfa2.putc(command);
            edfa2.putc(MSB);
            edfa2.putc(LSB);
            break;
        }
    }
    else if (command == S_OUTPUT_POWER){
        char MSB,LSB;
        LSB = data & 0x00FF;
        MSB = data >> 8;
        switch(edfa){
            case 1:
            edfa1.putc(S_RUNNING_MODE); // Set Running Mode
            edfa1.putc(0x00); // APC - Automatic Output Pump Control Mode
            edfa1.putc(command);  
            edfa1.putc(MSB);
            edfa1.putc(LSB);
            break;
            
            case 2:
            edfa2.putc(S_RUNNING_MODE); // Set Running Mode
            edfa2.putc(0x00); // APC - Automatic Output Pump Control Mode
            edfa2.putc(command);  
            edfa2.putc(MSB);
            edfa2.putc(LSB);
            break;
        }
    }
    else{    
        //pc.printf("Not a valid command for setting.");
    }        
}    
//______________________________________________________________________________

uint16_t read_edfa(uint8_t command,int edfa){
    if (command==R_PUMP_CURRENT || command==R_OUTPUT_POWER || command==R_PUMP_POWER || command==R_TEMPERATURE){
        char MSB,LSB;
        switch(edfa){
            case 1:
            edfa1.putc(command); // Send read command
            MSB = edfa1.getc();  // Get MSB
            LSB = edfa1.getc();  // Get LSB
            break;
            
            case 2:
            edfa2.putc(command); // Send read command
            MSB = edfa2.getc();  // Get MSB
            LSB = edfa2.getc();  // Get LSB
            break;
        }    
        return LSB|(MSB<<8); // Get Data
    }
    else if (command==R_ALARM || command==R_RUNNING_MODE){
        uint16_t X;
        switch(edfa){
            case 1:
            edfa1.putc(command); // Send read command
            X = edfa1.getc();
            break;
            
            case 2:
            edfa2.putc(command); // Send read command
            X = edfa2.getc();
            break;
        }
        return X;       
    }
    
    else{
        //pc.printf("Not a valid command for reading.");
        return 0;
    }         
}

//______________________________________________________________________________

void display(int page, bool blink){
    
    lcd.cls();
    
    switch(page){
        //PAGE 1//        
        case 1:
            if(blink){
                if(set_line){
                    if (set_stage==1)lcd.printf("1-PUMP: 000 mA  ");
                    if (set_stage==2)lcd.printf("1-PUMP: %03d mA  ",pump_value);     
                }else{
                    lcd.locate(0,1);   
                    if (set_stage==1)lcd.printf("1-OUT:  000 dBm");
                    if (set_stage==2)lcd.printf("1-OUT:  %03d dBm",out_value); 
                }
            }else{    
                lcd.printf("1-PUMP: %03d mA  ",read_edfa(R_PUMP_CURRENT,1)/10);
                lcd.printf("1-OUT:  %03d dBm",read_edfa(R_OUTPUT_POWER,1)/100);
            }    
            break;
        
        //PAGE 2//        
        case 2:
            lcd.printf("1-TEMP: %02d C    ",read_edfa(R_TEMPERATURE,1));
            lcd.printf("1-ALARM: ");
            print_alarm(1);
            break;
        //PAGE 3//        
        case 3:
            if(blink){
                if(set_line){
                    if (set_stage==1)lcd.printf("2-PUMP: 000 mA  ");
                    if (set_stage==2)lcd.printf("2-PUMP: %03d mA  ",pump_value);     
                }else{
                    lcd.locate(0,1);   
                    if (set_stage==1)lcd.printf("2-OUT:  000 dBm");
                    if (set_stage==2)lcd.printf("2-OUT:  %03d dBm",out_value); 
                }
            }else{    
                lcd.printf("2-PUMP: %03d mA  ",read_edfa(R_PUMP_CURRENT,2)/10);
                lcd.printf("2-OUT:  %03d dBm",read_edfa(R_OUTPUT_POWER,2)/100);
            }    
            break;
        //PAGE 4//        
        case 4:
            lcd.printf("2-TEMP: %02d C    ",read_edfa(R_TEMPERATURE,2));
            lcd.printf("2-ALARM: ");
            print_alarm(2);
            break;     
        //PAGE 5//        
        case 5:
            if(!send_pulse) lcd.printf("Press to start.");
            else if(keep_dc)
                {
                lcd.printf("Sending Pulses. ");
                lcd.printf("nloop: %d km",loop_number*fiber_spans*50);
                }
            else
                {
                 lcd.printf("Keep DC.        ");
                 lcd.printf("nloop: %02d",loop_number);
                }
            break;      
   }  
   
   wait_ms(100);
     
}  
    
//______________________________________________________________________________

void print_alarm(int edfa){
        uint16_t alarm;
        switch(edfa){
            case 1:
            alarm = read_edfa(R_ALARM,1);
            break;
            
            case 2:
            alarm = read_edfa(R_ALARM,2);
            break;
        }
        
        if(0x10&alarm) lcd.printf("1");
        else lcd.printf("0");
        if(0x08&alarm) lcd.printf("1");
        else lcd.printf("0");
        if(0x04&alarm) lcd.printf("1");
        else lcd.printf("0");
        if(0x02&alarm) lcd.printf("1");
        else lcd.printf("0");
        if(0x01&alarm) lcd.printf("1");
        else lcd.printf("0");
}    

//______________________________________________________________________________
  
    void A_rise(){
      up = true;
    }

    void button_rise(){
      press = true;
    }    
    
//______________________________________________________________________________    
    
    void debounce() {
        
        // PULSE CONTROL
        pulse_timer++; // Pulse Timer
        if (pulse_timer>=1000) pulse_timer=1000; // Stay ready for pulse
        
        // PAGE CONTROL
        deb <<= 1;
        deb = deb | rotary_A;
        if(up && deb==0xFF){
            
            if(rotary_A != rotary_B){
                if(set_stage==0) page++;
                else if(set_stage==1) set_line = !set_line;
                else if(set_stage==2 && set_line) pump_value=pump_value+1;
                else if(set_stage==2 && !set_line) out_value--;
                else if(set_stage==3) loop_number++;
                up = false;
            }else{
                if(set_stage==0) page--;
                else if(set_stage==1) set_line = !set_line;
                else if(set_stage==2 && set_line) pump_value=pump_value-1;
                else if(set_stage==2 && !set_line) out_value++;
                else if(set_stage==3) loop_number--;
                up = false;
            }    
        }   
        if(page>5)page=1;
        if(page<1)page=5;
        if(pump_value>300) pump_value=300;
        if(pump_value<0) pump_value=0;
        if(loop_number>50) loop_number=50;
        if(loop_number<1) loop_number=1;
        
        // BUTTON CONTROL
        if(button && press && (page==1 || page==3 || page==5)){
            pressed_time++;
        }else{
            pressed_time=0;
            press = false;
        }        
        
        if(pressed_time>100 && (page==1 || page==3) ){
            set_stage++;
            pressed_time = 0;
            press = false;    
        }
        
        if(pressed_time>100 && page==5 ){
            if(!keep_dc){
                send_pulse = !send_pulse;
                keep_dc = true;
                set_stage=0;
            }else{
                keep_dc = false;
                if(send_pulse) set_stage=3;
            }
            pressed_time = 0;
            press = false;    
        }
   
            
    }   