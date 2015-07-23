// ECE 4760 Final Project: FFT MCU Code
// Alexander Wang (ajw89) and Bill Jo (bwj8)
//
// Samples audio input using ADC and converts it into
// the frequency domain using a fixed-point FFT
// and then transmits the data to the Video MCU.

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h> 
#include <stdio.h>
#include <math.h> 
#define F_CPU 16000000UL
#include <util/delay.h>  
#include <avr/sleep.h>

// optional, if preferred//
#define begin {
#define end   }

//------------Start of borrowed code from Bruce Land--------------//
#define int2fix(a)   (((int)(a))<<8) 
#define float2fix(a) ((int)((a)*256.0)) 
#define fix2float(a) ((float)(a)/256.0)  

// Fast fixed point multiply assembler macro
#define multfix(a,b)          	  \
({                                \
int prod, val1=a, val2=b ;        \
__asm__ __volatile__ (            \ 
"muls %B1, %B2	\n\t"              \
"mov %B0, r0 \n\t"	               \ 
"mul %A1, %A2\n\t"	               \ 
"mov %A0, r1   \n\t"              \ 
"mulsu %B1, %A2	\n\t"          \ 
"add %A0, r0  \n\t"               \ 
"adc %B0, r1 \n\t"                \ 
"mulsu %B2, %A1	\n\t"          \ 
"add %A0, r0 \n\t"           \ 
"adc %B0, r1  \n\t"          \ 
"clr r1  \n\t" 		         \ 
: "=&d" (prod)               \
: "a" (val1), "a" (val2)      \
);                            \
prod;                        \
})

#define N_WAVE          128    /* size of FFT */
#define LOG2_N_WAVE     7     /* log2(N_WAVE) */	
//------------End of borrowed code from Bruce Land--------------//

//ISR timing
#define ADC_TIME 2000 	// 2000 cycles = 125us * 16MHz
#define SLEEP_TIME 1975 // set SLEEP to occur slightly before ISR

//FFT buffer
#define spectrum_bins 32 			// amount of bins to send/display
int fftarray[N_WAVE];				// array to hold FFT points
char specbuff[spectrum_bins];		// array to hold freq bin data to transmit
char erasespecbuff[spectrum_bins];	// empty array to clear spec buff
unsigned char currbin;				// index of specbuff

// ADC Variables
volatile signed int adcbuff[N_WAVE];	// array to hold ADC audio sample points
volatile char adcind;					// index of adcbuff
int adcMask[N_WAVE];					// trapezoidal windowing function for ADC buffer

// State Machine Variables
#define Release 0
#define Debounce 1
#define Pressed 2
#define DebounceRel 3
#define Toggle 4

// Button FSM
unsigned char freqState; // State variable for freqScaleFSM
unsigned char buttons;	 // stores value of PINC

// User options
char freqopt;			// frequency scale option

//function declarations
void freqScaleFSM(void);	// state machine function for freq scale select option button

int Sinewave[N_WAVE]; // a table of sines for the FFT	

signed int fr[N_WAVE],fi[N_WAVE],erasefi[N_WAVE];	// arrays used by FFT to store real, imaginary data, and a blank erase array

// put the MCU to sleep JUST before the CompA ISR goes off to ensure precise timing
ISR(TIMER1_COMPB_vect, ISR_NAKED)
{
	sei();
	sleep_cpu();
	reti();
}

//==================================
//This is ADC sampling of the audio signal. It MUST be entered from 
//sleep mode to get accurate timing of samples.

//run this every 125 us for every ADC sample (8 kHz sampling rate, 4 kHz max freq range without aliasing)
ISR (TIMER1_COMPA_vect) {
	if(adcind<N_WAVE) {	// if ADC buffer isn't full...
		//store an ADC sample and start the next one
		adcbuff[adcind++]=ADCH-140;		// subtract 140 to remove DC offset, corresponds to about 1.4V
		ADCSRA |= (1<<ADSC);
	}
}

//------------Start of borrowed code from Bruce Land--------------//
//===================================
//FFT function
void FFTfix(int fr[], int fi[], int m)
//Adapted from code by:
//Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
//fr[n],fi[n] are real,imaginary arrays, INPUT AND RESULT.
//size of data = 2**m 
// This routine does foward transform only
begin 
    int mr,nn,i,j,L,k,istep, n;
    int qr,qi,tr,ti,wr,wi; 
    
    mr = 0;
    n = 1<<m;   //number of points
    nn = n - 1;

    /* decimation in time - re-order data */
    for(m=1; m<=nn; ++m) 
    begin
        L = n;
        do L >>= 1; while(mr+L > nn);
        mr = (mr & (L-1)) + L;
        if(mr <= m) continue;
        tr = fr[m];
        fr[m] = fr[mr];
        fr[mr] = tr;
        //ti = fi[m];   //for real inputs, don't need this
        //fi[m] = fi[mr];
        //fi[mr] = ti;
    end
    
    L = 1;
    k = LOG2_N_WAVE-1;
    while(L < n) 
    begin
        istep = L << 1;
        for(m=0; m<L; ++m) 
        begin
            j = m << k;
            wr =  Sinewave[j+N_WAVE/4];
            wi = -Sinewave[j];
            wr >>= 1;
            wi >>= 1;
            
            for(i=m; i<n; i+=istep) 
            begin
                j = i + L;
                tr = multfix(wr,fr[j]) - multfix(wi,fi[j]);
                ti = multfix(wr,fi[j]) + multfix(wi,fr[j]);
                qr = fr[i] >> 1;
                qi = fi[i] >> 1;     
                fr[j] = qr - tr;
                fi[j] = qi - ti;
                fr[i] = qr + tr;
                fi[i] = qi + ti;
            end
        end
        --k;
        L = istep;
    end
end
//------------End of borrowed code from Bruce Land--------------//

//===================================
//Frequency Scale Select Button Press Debounce FSM

void freqScaleFSM(void){
	switch(freqState){
	    //starting state
		case Release:
			if ( (buttons & (1<<0)) == (1<<0)) freqState = Debounce;
			else freqState=Release;
		break;
		// see if button is still pressed
		case Debounce:
		    if((buttons & (1<<0)) == (1<<0)) freqState=Pressed;
        	else freqState=Release;
		break;
		// wait for release
		case Pressed:
		    if ((buttons & (1<<2)) == (1<<0)) freqState=Pressed; 
        else freqState=DebounceRel;
		break;
		// see if button is still released 
		case DebounceRel:
		    if ((buttons & (1<<0)) == (1<<0)) freqState=Pressed; 
			else freqState=Toggle;
		break;
		// toggle user option value
		case Toggle:
			freqState=Release;
			 // Toggle user option and signal to other MCU
			if (freqopt == 1) {freqopt = 0; PORTB &= ~(1<<PORTB3);}
			else {freqopt = 1; PORTB |= (1<<PORTB3);}
		break;
	}
}

//==================================         
// set up the ports and timers
int main() {
  //init timer 1 to sample audio
  // TIMER 1: OC1* disconnected, CTC mode, fosc/1 (16MHz), OC1A and OC1B interrupts enabled
  TCCR1B = _BV(WGM12) | _BV(CS10);
  OCR1A = ADC_TIME;	// time for one ADC sample
  OCR1B = SLEEP_TIME;	// time to go to sleep
  TIMSK1 = _BV(OCIE1B) | _BV(OCIE1A);

  //init ports
  DDRD |= (1<<DDD6) | (1<<DDD1);		// USART Ports to transmit to other microcontroller
  PORTD = 0x00;							// Turn off pull-up resistors
  DDRC = 0x00;							// push button ports
  PORTC = 0x01;							// pull up resistors on ports with buttons
  DDRB |= (1<<DDB3);					// push button "acknowledge" output signal to other MCU
  PORTB |= (1<<PORTB3);					// default to high output

  // USART in Synchronous mode, transmitter enabled, frequency 2Mbps
  DDRB |= (1<<DDB0);									// Enable output on USART0 Clock Pin
  UCSR0B = _BV(TXEN0);									// Enable transmit
  UCSR0C = _BV(UMSEL00) | (1<<UCSZ01) | (1<<UCSZ00);	// Enable USART Synchronous Mode with 8-bit character size
  UBRR0L = 2 ;											// Set transmit rate to 2 Mbps

  ///////////////////////
  // Set up the ADC
  ADMUX = (1<<ADLAR)|(1<<REFS1)|(1<<REFS0)+0;				// Enable ADC Left Adjust Result and 2.56V Voltage Reference and ADC Port 0
  ADCSRA = ((1<<ADEN)|(1<<ADSC))+7; 						// Runs at 125kHz, corresponds to 8-bit precision
  adcind=0;		// initialize array indexes
  currbin=0;

  // Buttons
  freqopt=1;	//set frequency range to 2 kHz initially
  freqState = Release;
  
  //loop iterator
  int i;
  // generate arrays
  for (i=0; i<N_WAVE; i++) {
    // Set up FFT, one cycle sine table required for FFT
    Sinewave[i] = float2fix(sin(6.283*((float)i)/N_WAVE)); 
	// generate empty array to erase
	erasefi[i]=0;
	// generate trapezoid mask (with 1/4 length slopes) for ADC buffer
	if(i<32) adcMask[i] = float2fix((8*(float)i/255));
	else if(i >= 32 && i <= 96) adcMask[i] = 0x0100;
	else if(i > 96) adcMask[i] = float2fix(((128-(float)i)*8/255));
  }
  // generate empty array to erase
  for (i=0; i<spectrum_bins; i++)
	erasespecbuff[i]=0;

  // Set up single ADC timing with sleep mode
  sei();
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();

  while(1) {
	// store Port C and update button press FSM
    buttons=PINC;
	freqScaleFSM();
	// if ADC buffer is full...
  	if (adcind >= N_WAVE) {
		// clear FFT arrays
		memcpy(specbuff,erasespecbuff,spectrum_bins);
		memcpy(fi,erasefi,N_WAVE);
		// copy ADC buffer into separate array
		memcpy(fr,adcbuff,N_WAVE);
		//scale the ADC values up for fixed point operation, and window with trapezoid with 32-pt slopes
		for(i=0; i<N_WAVE; i++){
			fr[i] = multfix((fr[i]<<4),adcMask[i]);
		}
		//do an 128 pt FFT here
		//save the magnitude of the the first 64 pts of the FFT into array (since all real input is reflected)
		FFTfix(fr, fi, LOG2_N_WAVE);
		for (i=0;i<(N_WAVE/2);i++) {
			//Magnitude Function: Sum of Squares of the Real & Imaginary parts
			fftarray[i]=multfix(fr[i],fr[i])+multfix(fi[i],fi[i]);
			//store 8-bit values into 32 frequency bins depending on overall frequency range
			if (freqopt==0) specbuff[(char)(i/2)]+=(char)(fftarray[i]);
			else if (freqopt==1 && i<32) specbuff[i]+=(char)(fftarray[i]);
		}
		//Transmit the 32 bytes of binned frequency data over to Video MCU
		//send Tx ready signal
		PORTD |= (1<<PORTD6);
		//Transmit in 4 byte packets as soon as Rx ready
		for (int j=0; j<8; j++) {
			//wait for Rx ready signal
			while ((PIND & (1<<PIND7)) != (1<<PIND7));
			for (i=0; i<4; i++) {
				while (!(UCSR0A & _BV(UDRE0))) ;
				UDR0 = specbuff[currbin++] ;
    	    }
		}
		//send Tx not ready signal after transmit complete
		while (!(UCSR0A & _BV(TXC0)));
		PORTD &= ~(1<<PORTD6);
		//reset array indexes to start acquiring data again
		currbin=0;
		adcind=0;
	}  //if
  }  //while
}  //main
