/*
 * EasyEVSE_FW_v10.c
 *
 * Created: 12/23/2020 5:50:05 PM
 * Author : USERNAME
 *
 * IMPORTANT: Don't forget to disable CKDIV8 fuse when programming new MCU
 *
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#define FWVERSIONDIGIT1 1
#define FWVERSIONDIGIT2 0
#define FWVERSIONDIGIT3 0

#define CURRENTSETLIMITLOW 13 //~10.4% duty cycle, ~6.24Amp
#define CURRENTSETLIMITHIGH 31 // 21 = ~17% duty cycle, ~10Amp // 31 = ~24.8% duty cycle, ~14.88Amp

//#define CURRENTSETLIMITHIGH 31 // ~24.8% duty cycle, ~14.88Amp
//#define CURRENTSETLIMITHIGH 67 // ~53.6% duty cycle, ~32.16Amp
//#define CURRENTSETLIMITHIGH 113 // ~90.4% duty cycle, ~66Amp

#define PILOTDUTYTOTAL 124
#define PILOTDUTYTOPERCENT 82 //~10000/(PILOTDUTYTOTAL-2)

#define RELATIVETIME TCNT1>>8 //32.768ms

// STATUS REGISTER
volatile uint8_t StatusRegister = 0;

#define STATUSEVCONNECTED 128
#define STATUSPILOTPWMENABLED 64
#define STATUSEVCHARGING 32
#define STATUSTIMERENABLED 16 //For future use
#define STATUSTIMERFINISHED 8 //For future use
#define STATUSRESERVED4 4 //For future use
#define STATUSRESERVED2 2 //For future use
#define STATUSAMPSETTINGMODE 1 //For future use


// ERROR REGISTER
volatile uint8_t ErrorRegister = 0;

#define ERRORINPUTVOLTAGE 128 //For future use
#define ERRORINTERNALVOLTAGE 64 //For future use
#define ERRORINTERNALTEMPERATURE 32
#define ERRORGFCI 16 //For future use
#define ERRORRELAY 8 //For future use
#define ERRORDIODE 4
#define ERRORVENTILATIONREQUIRED 2
#define ERRORPILOTHOUTOFRANGE 1

#define RELAYTIMEOUT 10000 //time in ms, when Relay can be turned on after turn off caused by Error (max ~ 64000 !)
#define RELAYDELAY 1000 //time in ms, when Relay can be turned on after turn off during normal condition e.g. unplug/plug to car. MUST be smaller than RELAYTIMEOUT

#define SETPILOTH PORTB |= 1<<PB2
#define SETPILOTL PORTB &= ~(1<<PB2)
#define PILOTISHIGH PINB & (1<<PB2)

#define SETRELAYON PORTD |= 1<<PD7
#define SETRELAYOFF PORTD &= ~(1<<PD7)
#define RELAYISON PIND & (1<<PD7)

#define SETLEDON PORTC |= 1<<PC3
#define SETLEDOFF PORTC &= ~(1<<PC3)
#define LEDISON PINC & (1<<PC3)

#define SETDEBUGH PORTD |= 1<<PD4
#define SETDEBUGL PORTD &= ~(1<<PD4)


#define ADMUXTEMPERATURE 0x60
#define ADMUXPILOT 0x61
#define ADMUXCURRENTRES 0x62
#define ADMUXMCUTEMP 0x68 //NOT USABLE !!!
#define ADMUXBANDGAP 0x6E //NOT USABLE !!!
#define ADMUXGND 0x6F

#define TEMPERATUREPOWEROFF 219 //2.822V ~75deg C
#define TEMPERATUREPOWERON 206 //2.655V ~65deg C

#define VOLTAGEEVUNPLUGGED 215 //215 to 255 means EV unplugged
#define VOLTAGEEVREADY 185 //185 to 214 means EV plugged and ready
#define VOLTAGEEVCHARGING 155 //155 to 184 means EV is charging
#define VOLTAGEEVVENTILATIONNEEDED 125 //125 to 154 means EV require ventilation
#define VOLTAGEDIODEOKMAX 20 //Voltage on negative pulse must be less than 20



#define FOSC 8000000 // Clock Speed
#define BAUD 38400 //baudrate
#define MYUBRR FOSC/16/BAUD-1

#define ASCIILF 10 //ASCII new line = /l
#define ASCIICR 13 //ASCII carriage return =/n
#define ASCIISPACE 32 //ASCII space
#define ASCIINUMBEROFSET 48 //ASCII NUMBER OFSET
#define ASCIISTX 2 //ASCII start of text

volatile uint8_t PilotDutyON = CURRENTSETLIMITLOW;
volatile uint8_t ADCpilotHIGH = 255;
volatile uint8_t ADCpilotLOW = 0;
volatile uint16_t RelayTimeout = RELAYTIMEOUT;

uint8_t Temperature;
uint8_t CurrentSetting;
uint8_t Temporary;
uint8_t USARTcounter=0;
uint8_t PilotDutyInPercent=0;
uint8_t PilotDutyInAmpere=0;

void HWinit()
{
	//USART INIT
	UBRR0H = ((MYUBRR)>>8);
	UBRR0L = MYUBRR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	
	
	//debug pin
	DDRD |= (1<<PD4); //set PD4 as output
	
	
	//PILOTMCU (1kHz output from MCU) @ PB2
	DDRB |= (1<<PB2); //set PB2 as output
	PORTB &= ~(1<<PB2); //set PB2 LOW
	
	//Relay output
	DDRD |= (1<<PD7); //set PD7 as output
	PORTD &= ~(1<<PD7); //set PD7 LOW
	
	//status LED
	DDRC |= (1<<PC3); //set PC3 as output
	PORTC &= ~(1<<PC3); //set PC3 LOW
	
	
	//DIGITAL INPUT DISABLE
	
	DIDR0 = 0x07; //ADC0-2 input disable
		
	//ADC enable
	//ADCSRA |= (1<<ADPS1) | (1<<ADPS0); //set ADC prescaler to 8 was not as stable as prescaler @ 16.
	ADCSRA |= (1<<ADPS2); //set ADC prescaler to 16. @8MHz RC osc it means 500kHz, ADC conversion takes 13cycles = about 26us or 25cycles (50us) for first conversion
	ADMUX = ADMUXGND;
	ADCSRA |= (1<<ADEN); //enable ADC module
	//PRR &= ~(1<<PRADC); //disable power reduction (enable ADC)
		//set ADSC bit in ADCSRA to start conversion, When the conversion is complete, it returns to zero.
	
	
	//Timer2 (TCNT2) for pilot signal, using 2 interrupts on compare A & B
	TIMSK2 |= (1<<OCF2A) | (1<<OCF2B); //enable interrupt on compare 2A and 2B
	TCCR2B |= (1<<CS22); //Start timer2 with CLK/64 prescaler = 8us/step, 2048us/OVF
	
	//Timer1 for relative time (LED blink)
	TCCR1B |= (1<<CS10)|(1<<CS12); //prescaler 1024 = 128us/step in TCNT1L, 32.768ms/ value change in TCNT1H, 8.3sec to ovf
}


ISR (TIMER2_COMPA_vect) //TCNT2 == OCR2A, End of positive pulse (OCR2A = PilotDutyON;)
{
	//SETDEBUGH;
	
	ADCSRA &= ~(1<<ADEN); //DISABLE ADC to terminate any conversion in progress, next measurement will take about 55us
	ADMUX = ADMUXPILOT; //set ADC MUX to read pilot voltage on ADC1
	ADCSRA |= (1<<ADEN) | (1<<ADSC); //ENABLE ADC and start conversion
	while (ADCSRA & (1<<ADSC)); //wait for conversion to finish
	//if (PILOTISHIGH) ADCpilotHIGH = ADCH; //if pilot is really HIGH, save pilot HIGH ADC value
	if (PILOTISHIGH && (ADCH > ADCpilotHIGH+1 || ADCH < ADCpilotHIGH-1)) ADCpilotHIGH = ADCH; //if pilot is really HIGH, save pilot HIGH ADC value if it change +/-2 (makes nice filtering)
	if (StatusRegister & STATUSPILOTPWMENABLED) SETPILOTL;	//set pilot LOW if PWM should be enabled
	
	//SETDEBUGL;
}

ISR (TIMER2_COMPB_vect) //TCNT2 == OCR2B, End of period (end of negative pulse) (OCR2B = PILOTDUTYTOTAL;)
{
	//SETDEBUGH;
	
	TCNT2 = 0; //Reset Timer2
	if ((!(RELAYISON)) && (RelayTimeout < RELAYTIMEOUT)) RelayTimeout++; //if relay is OFF and defined relay timeout is not reached, increase RelayTimeout
	
	ADCSRA &= ~(1<<ADEN); //DISABLE ADC to terminate any conversion in progress, next measurement will take about 55us
	ADMUX = ADMUXPILOT;  //set ADC MUX to read pilot voltage on ADC1
	ADCSRA |= (1<<ADEN) | (1<<ADSC); //ENABLE ADC and start conversion
	while (ADCSRA & (1<<ADSC)); //wait for conversion to finish
	//if (!(PILOTISHIGH)) ADCpilotLOW = ADCH; //if pilot is really LOW, save pilot LOW ADC value
	if (!(PILOTISHIGH) && (ADCH > ADCpilotLOW+1 || ADCH < ADCpilotLOW-1)) ADCpilotLOW = ADCH; //if pilot is really LOW, save pilot LOW ADC value if it change +/-2 (makes nice filtering)
	SETPILOTH; 	//set pilot HIGH
	
	//SETDEBUGL;
}

void USART_TX (uint8_t TXpayload)
{
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = TXpayload;
}

void SendData()
{
	if (UCSR0A & (1<<UDRE0)) //if USART TX buffer is Empty
	{
		switch(USARTcounter)
		{
			
			#define SENDDATAOFFSETTEMPERATURE 0
			#define SENDDATAOFFSETDUTY 10
			#define SENDDATAOFFSETDUTYPERCENTAGE 20
			#define SENDDATAOFFSETDUTYAMPERE 30			
			#define SENDDATAOFFSETRELAY 40
			#define SENDDATAOFFSETPILOTH 50
			#define SENDDATAOFFSETPILOTL 60
			#define SENDDATAOFFSETSTATUS 70
			#define SENDDATAOFFSETEVCON 80
			#define SENDDATAOFFSETPWMEN 90
			#define SENDDATAOFFSETEVCHARGE 100
			#define SENDDATAOFFSETERROR 110
			#define SENDDATAOFFSETERRPILOT 120
			#define SENDDATAOFFSETERRVENT 130
			#define SENDDATAOFFSETERRDIO 140
			#define SENDDATAOFFSETERRTEMP 150
			
			
			//TEMPERATURE
			case  SENDDATAOFFSETTEMPERATURE:
			UDR0 = ('T');
			break;
			
			case  SENDDATAOFFSETTEMPERATURE+1:
			UDR0 = ('e');
			break;
			
			case  SENDDATAOFFSETTEMPERATURE+2:
			UDR0 = ('m');
			break;
			
			case  SENDDATAOFFSETTEMPERATURE+3:
			UDR0 = ('p');
			break;
			
			case  SENDDATAOFFSETTEMPERATURE+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETTEMPERATURE+5:
			UDR0 = (Temperature/100%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETTEMPERATURE+6:
			UDR0 = (Temperature/10%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETTEMPERATURE+7:
			UDR0 = (Temperature%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETTEMPERATURE+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETTEMPERATURE+9:
			UDR0 = (ASCIICR);
			break;
			
			//DUTY
			case SENDDATAOFFSETDUTY:
			UDR0 = ('D');
			break;
			
			case SENDDATAOFFSETDUTY+1:
			UDR0 = ('u');
			break;
			
			case SENDDATAOFFSETDUTY+2:
			UDR0 = ('t');
			break;
			
			case SENDDATAOFFSETDUTY+3:
			UDR0 = ('y');
			break;
			
			case SENDDATAOFFSETDUTY+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETDUTY+5:
			UDR0 = (PilotDutyON/100%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETDUTY+6:
			UDR0 = (PilotDutyON/10%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETDUTY+7:
			UDR0 = (PilotDutyON%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETDUTY+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETDUTY+9:
			UDR0 = (ASCIICR);
			break;
			
			//DUTY percentage
			case  SENDDATAOFFSETDUTYPERCENTAGE:
			UDR0 = ('D');
			break;
			
			case  SENDDATAOFFSETDUTYPERCENTAGE+1:
			UDR0 = ('u');
			break;
			
			case  SENDDATAOFFSETDUTYPERCENTAGE+2:
			UDR0 = ('t');
			break;
			
			case  SENDDATAOFFSETDUTYPERCENTAGE+3:
			UDR0 = ('y');
			break;
			
			case  SENDDATAOFFSETDUTYPERCENTAGE+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETDUTYPERCENTAGE+5:
			UDR0 = (PilotDutyInPercent/10%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETDUTYPERCENTAGE+6:
			UDR0 = (PilotDutyInPercent%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETDUTYPERCENTAGE+7:
			UDR0 = ('%');
			break;
			
			case SENDDATAOFFSETDUTYPERCENTAGE+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETDUTYPERCENTAGE+9:
			UDR0 = (ASCIICR);
			break;
			
			//DUTY Ampere
			case  SENDDATAOFFSETDUTYAMPERE:
			UDR0 = ('D');
			break;
			
			case  SENDDATAOFFSETDUTYAMPERE+1:
			UDR0 = ('u');
			break;
			
			case  SENDDATAOFFSETDUTYAMPERE+2:
			UDR0 = ('t');
			break;
			
			case  SENDDATAOFFSETDUTYAMPERE+3:
			UDR0 = ('y');
			break;
			
			case  SENDDATAOFFSETDUTYAMPERE+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETDUTYAMPERE+5:
			UDR0 = (PilotDutyInAmpere/10%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETDUTYAMPERE+6:
			UDR0 = (PilotDutyInAmpere%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETDUTYAMPERE+7:
			UDR0 = ('A');
			break;
			
			case SENDDATAOFFSETDUTYAMPERE+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETDUTYAMPERE+9:
			UDR0 = (ASCIICR);
			break;
			
			//pilot H
			case SENDDATAOFFSETPILOTH:
			UDR0 = ('P');
			break;
			
			case SENDDATAOFFSETPILOTH+1:
			UDR0 = ('i');
			break;
			
			case SENDDATAOFFSETPILOTH+2:
			UDR0 = ('l');
			break;
			
			case SENDDATAOFFSETPILOTH+3:
			UDR0 = ('H');
			break;
			
			case SENDDATAOFFSETPILOTH+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETPILOTH+5:
			UDR0 = (ADCpilotHIGH/100%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETPILOTH+6:
			UDR0 = (ADCpilotHIGH/10%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETPILOTH+7:
			UDR0 = (ADCpilotHIGH%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETPILOTH+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETPILOTH+9:
			UDR0 = (ASCIICR);
			break;
			
			//pilot L
			case SENDDATAOFFSETPILOTL:
			UDR0 = ('P');
			break;
			
			case SENDDATAOFFSETPILOTL+1:
			UDR0 = ('i');
			break;
			
			case SENDDATAOFFSETPILOTL+2:
			UDR0 = ('l');
			break;
			
			case SENDDATAOFFSETPILOTL+3:
			UDR0 = ('L');
			break;
			
			case SENDDATAOFFSETPILOTL+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETPILOTL+5:
			UDR0 = (ADCpilotLOW/100%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETPILOTL+6:
			UDR0 = (ADCpilotLOW/10%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETPILOTL+7:
			UDR0 = (ADCpilotLOW%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETPILOTL+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETPILOTL+9:
			UDR0 = (ASCIICR);
			break;
									
			//Status register
			case SENDDATAOFFSETSTATUS:
			UDR0 = ('S');
			break;
			
			case SENDDATAOFFSETSTATUS+1:
			UDR0 = ('t');
			break;
			
			case SENDDATAOFFSETSTATUS+2:
			UDR0 = ('a');
			break;
			
			case SENDDATAOFFSETSTATUS+3:
			UDR0 = ('t');
			break;
			
			case SENDDATAOFFSETSTATUS+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETSTATUS+5:
			UDR0 = (StatusRegister/100%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETSTATUS+6:
			UDR0 = (StatusRegister/10%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETSTATUS+7:
			UDR0 = (StatusRegister%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETSTATUS+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETSTATUS+9:
			UDR0 = (ASCIICR);
			break;
			
			//Error register
			case SENDDATAOFFSETERROR:
			UDR0 = ('E');
			break;
			
			case SENDDATAOFFSETERROR+1:
			UDR0 = ('r');
			break;
			
			case SENDDATAOFFSETERROR+2:
			UDR0 = ('r');
			break;
			
			case SENDDATAOFFSETERROR+3:
			UDR0 = ('o');
			break;
			
			case SENDDATAOFFSETERROR+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETERROR+5:
			UDR0 = (ErrorRegister/100%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETERROR+6:
			UDR0 = (ErrorRegister/10%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETERROR+7:
			UDR0 = (ErrorRegister%10+ASCIINUMBEROFSET);
			break;
			
			case SENDDATAOFFSETERROR+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETERROR+9:
			UDR0 = (ASCIICR);
			break;
			
			
			//RELAY status
			case SENDDATAOFFSETRELAY:
			UDR0 = ('R');
			break;
			
			case SENDDATAOFFSETRELAY+1:
			UDR0 = ('e');
			break;
			
			case SENDDATAOFFSETRELAY+2:
			UDR0 = ('l');
			break;
			
			case SENDDATAOFFSETRELAY+3:
			UDR0 = ('a');
			break;
			
			case SENDDATAOFFSETRELAY+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETRELAY+5:
			UDR0 = ('O');
			break;
			
			case SENDDATAOFFSETRELAY+6:
			if (RELAYISON) UDR0 = ('N');
			else UDR0 = ('F');
			break;
			
			case SENDDATAOFFSETRELAY+7:
			if (RELAYISON) UDR0 = (ASCIISPACE);
			else UDR0 = ('F');
			break;
			
			case SENDDATAOFFSETRELAY+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETRELAY+9:
			UDR0 = (ASCIICR);
			break;
						
			//EV conn status
			case SENDDATAOFFSETEVCON:
			UDR0 = ('E');
			break;
			
			case SENDDATAOFFSETEVCON+1:
			UDR0 = ('V');
			break;
			
			case SENDDATAOFFSETEVCON+2:
			UDR0 = ('i');
			break;
			
			case SENDDATAOFFSETEVCON+3:
			UDR0 = ('s');
			break;
			
			case SENDDATAOFFSETEVCON+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETEVCON+5:
			if (StatusRegister & STATUSEVCONNECTED) UDR0 = ('C');
			else UDR0 = ('D');
			break;
			
			case SENDDATAOFFSETEVCON+6:
			if (StatusRegister & STATUSEVCONNECTED) UDR0 = ('O');
			else UDR0 = ('I');
			break;
			
			case SENDDATAOFFSETEVCON+7:
			if (StatusRegister & STATUSEVCONNECTED) UDR0 = ('N');
			else UDR0 = ('S');
			break;
			
			case SENDDATAOFFSETEVCON+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETEVCON+9:
			UDR0 = (ASCIICR);
			break;
			
			//PWM status
			case SENDDATAOFFSETPWMEN:
			UDR0 = ('P');
			break;
			
			case SENDDATAOFFSETPWMEN+1:
			UDR0 = ('W');
			break;
			
			case SENDDATAOFFSETPWMEN+2:
			UDR0 = ('M');
			break;
			
			case SENDDATAOFFSETPWMEN+3:
			UDR0 = ('s');
			break;
			
			case SENDDATAOFFSETPWMEN+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETPWMEN+5:
			UDR0 = ('O');
			break;
			
			case SENDDATAOFFSETPWMEN+6:
			if (StatusRegister & STATUSPILOTPWMENABLED) UDR0 = ('N');
			else UDR0 = ('F');
			break;
			
			case SENDDATAOFFSETPWMEN+7:
			if (StatusRegister & STATUSPILOTPWMENABLED) UDR0 = (ASCIISPACE);
			else UDR0 = ('F');
			break;
			
			case SENDDATAOFFSETPWMEN+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETPWMEN+9:
			UDR0 = (ASCIICR);
			break;
			
			//EV charge
			case SENDDATAOFFSETEVCHARGE:
			UDR0 = ('E');
			break;
			
			case SENDDATAOFFSETEVCHARGE+1:
			UDR0 = ('V');
			break;
			
			case SENDDATAOFFSETEVCHARGE+2:
			UDR0 = ('c');
			break;
			
			case SENDDATAOFFSETEVCHARGE+3:
			UDR0 = ('h');
			break;
			
			case SENDDATAOFFSETEVCHARGE+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETEVCHARGE+5:
			if (StatusRegister & STATUSEVCHARGING) UDR0 = ('Y');
			else UDR0 = ('N');
			break;
			
			case SENDDATAOFFSETEVCHARGE+6:
			if (StatusRegister & STATUSEVCHARGING) UDR0 = ('E');
			else UDR0 = ('O');
			break;
			
			case SENDDATAOFFSETEVCHARGE+7:
			if (StatusRegister & STATUSEVCHARGING) UDR0 = ('S');
			else UDR0 = (ASCIISPACE);
			break;
			
			case SENDDATAOFFSETEVCHARGE+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETEVCHARGE+9:
			UDR0 = (ASCIICR);
			break;
			
			//ERR pilot
			case SENDDATAOFFSETERRPILOT:
			UDR0 = ('E');
			break;
			
			case SENDDATAOFFSETERRPILOT+1:
			UDR0 = ('r');
			break;
			
			case SENDDATAOFFSETERRPILOT+2:
			UDR0 = ('P');
			break;
			
			case SENDDATAOFFSETERRPILOT+3:
			UDR0 = ('i');
			break;
			
			case SENDDATAOFFSETERRPILOT+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETERRPILOT+5:
			if (ErrorRegister & ERRORPILOTHOUTOFRANGE) UDR0 = ('Y');
			else UDR0 = ('N');
			break;
			
			case SENDDATAOFFSETERRPILOT+6:
			if (ErrorRegister & ERRORPILOTHOUTOFRANGE) UDR0 = ('E');
			else UDR0 = ('O');
			break;
			
			case SENDDATAOFFSETERRPILOT+7:
			if (ErrorRegister & ERRORPILOTHOUTOFRANGE) UDR0 = ('S');
			else UDR0 = (ASCIISPACE);
			break;
			
			case SENDDATAOFFSETERRPILOT+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETERRPILOT+9:
			UDR0 = (ASCIICR);
			break;			

			//ERR ventilation
			case SENDDATAOFFSETERRVENT:
			UDR0 = ('E');
			break;
			
			case SENDDATAOFFSETERRVENT+1:
			UDR0 = ('r');
			break;
			
			case SENDDATAOFFSETERRVENT+2:
			UDR0 = ('V');
			break;
			
			case SENDDATAOFFSETERRVENT+3:
			UDR0 = ('e');
			break;
			
			case SENDDATAOFFSETERRVENT+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETERRVENT+5:
			if (ErrorRegister & ERRORVENTILATIONREQUIRED) UDR0 = ('Y');
			else UDR0 = ('N');
			break;
			
			case SENDDATAOFFSETERRVENT+6:
			if (ErrorRegister & ERRORVENTILATIONREQUIRED) UDR0 = ('E');
			else UDR0 = ('O');
			break;
			
			case SENDDATAOFFSETERRVENT+7:
			if (ErrorRegister & ERRORVENTILATIONREQUIRED) UDR0 = ('S');
			else UDR0 = (ASCIISPACE);
			break;
			
			case SENDDATAOFFSETERRVENT+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETERRVENT+9:
			UDR0 = (ASCIICR);
			break;
			
			//ERR Diode
			case SENDDATAOFFSETERRDIO:
			UDR0 = ('E');
			break;
			
			case SENDDATAOFFSETERRDIO+1:
			UDR0 = ('r');
			break;
			
			case SENDDATAOFFSETERRDIO+2:
			UDR0 = ('D');
			break;
			
			case SENDDATAOFFSETERRDIO+3:
			UDR0 = ('i');
			break;
			
			case SENDDATAOFFSETERRDIO+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETERRDIO+5:
			if (ErrorRegister & ERRORDIODE) UDR0 = ('Y');
			else UDR0 = ('N');
			break;
			
			case SENDDATAOFFSETERRDIO+6:
			if (ErrorRegister & ERRORDIODE) UDR0 = ('E');
			else UDR0 = ('O');
			break;
			
			case SENDDATAOFFSETERRDIO+7:
			if (ErrorRegister & ERRORDIODE) UDR0 = ('S');
			else UDR0 = (ASCIISPACE);
			break;
			
			case SENDDATAOFFSETERRDIO+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETERRDIO+9:
			UDR0 = (ASCIICR);
			break;
			
			//ERR Temperature
			case SENDDATAOFFSETERRTEMP:
			UDR0 = ('E');
			break;
			
			case SENDDATAOFFSETERRTEMP+1:
			UDR0 = ('r');
			break;
			
			case SENDDATAOFFSETERRTEMP+2:
			UDR0 = ('T');
			break;
			
			case SENDDATAOFFSETERRTEMP+3:
			UDR0 = ('e');
			break;
			
			case SENDDATAOFFSETERRTEMP+4:
			UDR0 = (':');
			break;
			
			case SENDDATAOFFSETERRTEMP+5:
			if (ErrorRegister & ERRORINTERNALTEMPERATURE) UDR0 = ('Y');
			else UDR0 = ('N');
			break;
			
			case SENDDATAOFFSETERRTEMP+6:
			if (ErrorRegister & ERRORINTERNALTEMPERATURE) UDR0 = ('E');
			else UDR0 = ('O');
			break;
			
			case SENDDATAOFFSETERRTEMP+7:
			if (ErrorRegister & ERRORINTERNALTEMPERATURE) UDR0 = ('S');
			else UDR0 = (ASCIISPACE);
			break;
			
			case SENDDATAOFFSETERRTEMP+8:
			UDR0 = (ASCIILF);
			break;
			
			case SENDDATAOFFSETERRTEMP+9:
			UDR0 = (ASCIICR);
			break;
			
			case 160:
			UDR0 = (ASCIILF);
			break;
			
			
			case 161 ... 255:
			UDR0 = (ASCIICR);
			USARTcounter = 255;
			
			break;
			
			
		}
		USARTcounter++;
	}
		
}

void SendWelcomeMessage ()
{
	USART_TX(ASCIILF);
	USART_TX(ASCIICR);
	USART_TX('E');
	USART_TX('a');
	USART_TX('s');
	USART_TX('y');
	USART_TX('E');
	USART_TX('V');
	USART_TX('S');
	USART_TX('E');
	USART_TX(ASCIISPACE);
	USART_TX('F');
	USART_TX('w');
	USART_TX(FWVERSIONDIGIT1+48);
	USART_TX('.');
	USART_TX(FWVERSIONDIGIT2+48);
	USART_TX(FWVERSIONDIGIT3+48);
	USART_TX(ASCIILF);
	USART_TX(ASCIICR);
}


uint8_t	blink_on_duration_OLD = 0;
uint8_t	blink_off_duration_OLD = 0;
uint8_t blink_repeat_count_OLD = 0;
uint8_t blink_repeat_counter = 0;

uint8_t next_led_on_relativetime = 0;
uint8_t next_led_off_relativetime = 0;


void LEDblink (uint8_t blink_on_duration, uint8_t blink_off_duration, uint8_t blink_repeat_count) //takes about 6us
{
	uint8_t relativetime = RELATIVETIME; //save relative time to avoid changes in time
	
	if (blink_on_duration) //if LED should blink
	{
		if ((blink_on_duration != blink_on_duration_OLD) || (blink_off_duration != blink_off_duration_OLD) || (blink_repeat_count != blink_repeat_count_OLD)) //if we got new parameters
		{
			SETLEDON;	//turn LED on
			next_led_off_relativetime = (relativetime+blink_on_duration); //and set nearest off time
			next_led_on_relativetime = relativetime-1; //this is "reset" of next led on relative time
			if(blink_repeat_count > 0) blink_repeat_counter = 1;
			else blink_repeat_counter = 0;
		}
		
		else if (relativetime == next_led_off_relativetime && (LEDISON)) //if it is time to switch LED off and LED is on
		{
			SETLEDOFF; //turn LED off
			if (blink_repeat_count==0 || blink_repeat_counter>0) next_led_on_relativetime = (relativetime+blink_off_duration); //if led should blink infinite time or blink counter is >0  then  set nearest on time
		}
		
		else if ((relativetime == next_led_on_relativetime) && !(LEDISON) && (blink_repeat_counter <= blink_repeat_count)) //if it is time to switch LED on and	LED is off and blink counter is <= blink repeat count
		{
			SETLEDON; //turn LED on
			next_led_off_relativetime = (relativetime+blink_on_duration); //and set nearest off time
			if(blink_repeat_count > 0) blink_repeat_counter++; //if LED should blink finite times increase repeat counter
		}
		
		else if (blink_repeat_count > 0 && blink_repeat_count == blink_repeat_counter) //if LED should blink finite times and last blink started
		{
			//next_led_on_relativetime = relativetime-1; //"do break in blinks" about 8 sec
			next_led_on_relativetime = relativetime-180; //"do break in blinks" about 2.4sec
			blink_repeat_counter = 0; //reset counter
		}
		
		blink_on_duration_OLD = blink_on_duration;
		blink_off_duration_OLD = blink_off_duration;
		blink_repeat_count_OLD = blink_repeat_count;
	}
	else SETLEDOFF;
}

int main(void)
{
    HWinit();	
	OCR2A = PilotDutyON;
	OCR2B = PILOTDUTYTOTAL;
	SendWelcomeMessage();
	TCNT2 = 0; //Reset Timer2
	sei();
	while (1)
    {		
		
		//TEMPERATURE  measure (~30us), set / clear Error if needed
		ADMUX = ADMUXTEMPERATURE;
		ADCSRA |= (1<<ADSC); //Start ADC conversion		
		while (ADCSRA & (1<<ADSC)); //wait for conversion to finish
		Temporary = ADCH;
		if (ADMUX == ADMUXTEMPERATURE) Temperature = Temporary; //save Temperature value only if ADMUX was not changed in Interrupt
		if (Temperature >= TEMPERATUREPOWEROFF) ErrorRegister |= ERRORINTERNALTEMPERATURE; //Set Error when OVT
		else if (Temperature <= TEMPERATUREPOWERON) ErrorRegister &= ~ERRORINTERNALTEMPERATURE; //Reset OVT Error when Temperature drops
				
		//CURRENT SET RESISTOR measure (~30us)
		ADMUX = ADMUXCURRENTRES;
		ADCSRA |= (1<<ADSC); //Start ADC conversion	
		while (ADCSRA & (1<<ADSC)); //wait for conversion to finish
		Temporary = ADCH;
		if (ADMUX == ADMUXCURRENTRES) CurrentSetting = Temporary; //save desired Current setting value
		
		//Convert desired Current read to Pilot Duty ON cycle based on MIN/MAX values
		switch (CurrentSetting)
		{
			case 0 ... (CURRENTSETLIMITLOW):
			PilotDutyON = CURRENTSETLIMITLOW;
			break;
		
			case (CURRENTSETLIMITLOW+1) ... CURRENTSETLIMITHIGH:
			PilotDutyON = CurrentSetting;
			break;
		
			case (CURRENTSETLIMITHIGH+1) ... 255:
			PilotDutyON = CURRENTSETLIMITHIGH;
			break;				
		}
		if (PilotDutyON > OCR2A || PilotDutyON < OCR2A-1) OCR2A = PilotDutyON; //apply current setting to PILOT PWM only when value changed up by 1value, or down 2values (makes nice filtering)
		//OCR2A = PilotDutyON; //apply current setting to PILOT PWM
		
		
		//MAIN of MAIN :)
				
		switch (ADCpilotHIGH) //based on pilot HIGH voltage set status/error register-----------!! NO ERRORS ARE CLEARED HERE !!-------------
		{
			case VOLTAGEEVUNPLUGGED ... 255: //in case that EV is unplugged ------AKA STATE A----------
			
			SETRELAYOFF; //turn off relay
			RelayTimeout = RELAYTIMEOUT - RELAYDELAY; //set relay timeout for next turn on delay
			StatusRegister &= ~(STATUSEVCONNECTED|STATUSPILOTPWMENABLED|STATUSEVCHARGING); //clear these status bits		
			ADCpilotLOW = 0; //reset pilot LOW
			break;
			
			case VOLTAGEEVREADY ... (VOLTAGEEVUNPLUGGED-1): //in case that EV is plugged and ready ------AKA STATE B----------
			SETRELAYOFF; //turn off relay
			RelayTimeout = RELAYTIMEOUT - RELAYDELAY; //set relay timeout for next turn on delay
			if (!ErrorRegister) //ONLY if there are no Errors
			{
				StatusRegister |= STATUSEVCONNECTED | STATUSPILOTPWMENABLED; //set these status bits (STATUSPILOTPWMENABLED bit to StatusRegister start a PWM generator)
				StatusRegister &= ~(STATUSEVCHARGING); //clear this status bit
			}
			break;
			
			case VOLTAGEEVCHARGING ... (VOLTAGEEVREADY-1): //in case that EV is charging ------AKA STATE C----------
			if (!ErrorRegister) //ONLY if there are no Errors
			{
				StatusRegister |= STATUSEVCONNECTED | STATUSPILOTPWMENABLED | STATUSEVCHARGING; //set these status bits
				if (RelayTimeout>=RELAYTIMEOUT) SETRELAYON; //turn on Relay  //????????? RELAY ON HERE IS GOOD IDEA ??????????? When increasing ADCpilot from OUTof range / ventilation needed relay is off. But EV should stop charging as it has no power, and start from init procedure. So maybe is better to not set Relay ON in this stage. This is to be checked...
			}
			break;
			
			case VOLTAGEEVVENTILATIONNEEDED ... (VOLTAGEEVCHARGING-1): //in case that EV needs ventilation ------AKA STATE D----------
			ErrorRegister |= ERRORVENTILATIONREQUIRED; //set these error bits
			break;
			
			case 0 ... VOLTAGEEVVENTILATIONNEEDED-1: //in case pilot HIGH is out of range
			ErrorRegister |= ERRORPILOTHOUTOFRANGE; //set these error bits
			break;	
		}
		
		//DIODE CHECK
		
		if (ADCpilotLOW <= VOLTAGEDIODEOKMAX) ErrorRegister &= ~ERRORDIODE; //if ADCpilotLOW is lower than VOLTAGEDIODEOKMAX threshold it means diode check pass and diode error bit is cleared
		else ErrorRegister |= ERRORDIODE; //else there is diode error. NOTE: for diode error clear, EV must be unplugged, or EVSE restarted.
		
		
		if (ErrorRegister) // if there are Errors, set relay OFF, and check if errors still occur, if not clear them
			{
				SETRELAYOFF; //turn off relay
				RelayTimeout = 0; //reset relay timeout, so relay can be turned on only after timeout (to prevent high frequency relay switching)
				
				StatusRegister &= ~(STATUSPILOTPWMENABLED | STATUSEVCHARGING); //Disable PWM, clear charging status				
				
				//diode Error clear
				if (ADCpilotHIGH >= VOLTAGEEVUNPLUGGED) //If EV is unplugged clear diode Error and reset pilot LOW. NOTE: AS DIODE ERROR is cleared here only, EVSE needs to be unplugged from EV (or EVSE reset) to clear this diode error (after diode check error)
				{
					ErrorRegister &= ~ERRORDIODE;
					ADCpilotLOW = 0; 
				}
				//ventilation needed error clear
				if (ADCpilotHIGH>=VOLTAGEEVCHARGING) ErrorRegister &= ~ERRORVENTILATIONREQUIRED; //if ADC pilot HIGH is higher than voltage where ventilation is needed, clear ventilation needed error bit
				
				//pilot out of range error clear
				if (ADCpilotHIGH>=VOLTAGEEVVENTILATIONNEEDED) ErrorRegister &= ~ERRORPILOTHOUTOFRANGE; //if ADC pilot HIGH is higher at least on Ventilation needed level, clear pilot out of range error bit
				
			}
			
		//Approx Calculation of duty in percent ~30us
		uint16_t Temporary16bit = OCR2A*PILOTDUTYTOPERCENT/100;
		PilotDutyInPercent = Temporary16bit;
		
		
		//Approx Calculation of Ampere based on duty in percent ~30us
		if (PilotDutyInPercent<86)
		{
			Temporary16bit = PilotDutyInPercent*6/10;
			PilotDutyInAmpere = Temporary16bit;
		}
		else
		{
			Temporary16bit = (PilotDutyInPercent-64)*25/10;
			PilotDutyInAmpere = Temporary16bit;
		}
		
		//display status on LED
		if (ErrorRegister) LEDblink(3,3,ErrorRegister); //fast blink Error Status if there is error
		else if (!(StatusRegister & STATUSEVCONNECTED)) SETLEDOFF; //LED OFF if EV is disconnected
		else if ((StatusRegister & STATUSEVCONNECTED) && !(StatusRegister & STATUSEVCHARGING)) SETLEDON; //LED ON if EV is connected but not charging
		else if (StatusRegister & STATUSEVCHARGING) LEDblink(10,10,PilotDutyInAmpere); //if EV is charging, blink ampere setting
		
		SendData(); //max 30us
    }
}
