///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "msp430g2553.h"
#include "lcd.h"
#include "comun.h"
#include "stdio.h"

#define SI4735_ADDR	0x11                    // 0x22 W 0x23 W
#define FM 0x10
#define AM 0x11
#define SW 0x12
#define LW 0x13


/*
static const char poweronfm_inicio[]={0x01,0x10,0x05};
static const char rclk_inicio[]={0x12, 0x00, 0x02, 0x01, 0x80, 0x00};
static const char freq_inicio[]={0x20, 0x00, 0x27, 0x1A};
static const char volumen_inicio[]= {0x12, 0x00, 0x40, 0x00, 0x00, 0x3F};
static const char mute_inicio[]={0x12, 0x00, 0x40, 0x01, 0x00, 0x00};
*/


unsigned char _modo=FM, _cuenta, _ab, _newRadioText , _scroll, _scroll_j=0, _ps_rdy;
unsigned int freq=9400, refresca;
char temp[7];

// RDS

char _ps[8], _disp[64], _pty[16], _csign[4];


void start_TX(void);
void start_RX(void);
void sendBytes(unsigned char address, int bytes_to_tx);
void getBytes(unsigned char address, int bytes_to_rx);


void pantalla(void);
void si4735_cts( void );
char si4735_stcint( void );
void si4735_getREV( void );
void si4735_inicia(void);
void si4735_sintoniza( void );
void si4735_rds ( void );
void si4735_clearRDS(void) ;
void si4735_radiotext(void);

	//uart
void TXString( char* string );

	// iic
unsigned char *PTxData;                     	// Pointer to TX data
unsigned char *PRxData;                     	// Pointer to RX data

unsigned char TXByteCtr;
unsigned char RXByteCtr;

char command[8];
char response[16];



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void main(void)
{
    WDTCTL = WDTPW + WDTHOLD;  
    
    inicia();
    __enable_interrupt();           // Set global interrupt enable

    lcd_init();
    lcd_clear(0);
    lcd_print("Si4735",0,0);
    lcd_print("Iniciando I2C",0,1);

    si4735_inicia();
    lcd_clear(0);
    
    pantalla();

    TXString("Hola mundo!");

    print_int(89098,4);
 
    while(1){
      
        
     si4735_rds();

     if (!_newRadioText) {
    	 si4735_radiotext();
     }

     // __bis_SR_register(LPM0_bits + GIE);
      
      if (refresca){


        switch(_modo){
            case FM:
                sprintf(command, "%c%c", 0x21, 0x0C);
                sendBytes(SI4735_ADDR, 2);
                
                break;
            case AM:
            case SW:
            case LW:
                sprintf(command, "%c%c%c%c%c%c", 0x41, 0x0C, 0x00, 0x00, 0x00, 0x01);
                sendBytes(SI4735_ADDR, 6);
                
                break;
            default:
                break;
          }
      
        while ( !si4735_stcint() ) {
          
          if ( _cuenta > 1) {
            _cuenta=0;
            pantalla();
          }
          else
            _cuenta++;
        }
        
        refresca=0;        
        si4735_clearRDS();

      }
      
    }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void iic_tx_init(unsigned char address)
{
    UCB0I2CSA = address;							// Pone la dirección del esclavo en el bus
    IE2  |= UCB0TXIE;                       	// Enable TX interrupt
}
void iic_rx_init(void)
{
    IE2  |= UCB0RXIE;							// enable RX interrupt	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// send 1 byte, return 2 bytes - needs the return expanded for X bytes
void sendBytes(unsigned char address, int bytes_to_tx)
{

    // transmit slave address and register to read
    iic_tx_init(address);

    __delay_cycles(1000);

    PTxData = (unsigned char *)command;     // TX array start address

    TXByteCtr = bytes_to_tx;             // Load TX byte counter

    start_TX();
    
    si4735_cts();

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getBytes(unsigned char address, int bytes_to_rx)
{
  
    iic_tx_init(address);

    // receive requested bytes
    iic_rx_init();								// set RX interrupt
    
    __delay_cycles(1000);
    
    PRxData = (unsigned char *)response;    	// rx buffer
    
    RXByteCtr = bytes_to_rx;     				// number of bytes to receive
    
    start_RX();
    
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// iic start transmitting
void start_TX(void)
{
    UCB0CTL1 |= UCTR + UCTXSTT;             	// I2C TX, start condition
    
    __bis_SR_register(LPM0_bits + GIE);     	// Enter LPM0, enable interrupts
    __no_operation();                       	// Remain in LPM0 until all data
                                            	// is TX'd
    while (UCB0CTL1 & UCTXSTP);             	// Ensure stop condition got sent
 
}
    
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// iic restart and receive
void start_RX(void)
{
    while (UCB0CTL1 & UCTXSTP);             	// wait for stop
    
    UCB0CTL1 &= ~UCTR;                      	// restart, set as receiver
    UCB0CTL1 |= UCTXSTT;                    	// start condition
    
    __bis_SR_register(LPM0_bits + GIE);
    __no_operation();                       	// Remain in LPM0 until all data
    
    while (UCB0CTL1 & UCTXSTP);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TXString( char* string )
{
  
  while (*string!= '\0')
  {
    UCA0TXBUF = *string;
    string++;
    while (!(IFG2&UCA0TXIFG));              // USCI_A0 TX buffer ready?
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void si4735_cts( void ) {

  // Espera hasta que el bit MSB este a 1

  
  do {
      getBytes ( SI4735_ADDR,2);
      __delay_cycles(1000);
  }
  
  while (!(response[0] & 0x80));
  
  __delay_cycles(200000);
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
char si4735_stcint( void ) {

  // Espera hasta que el bit MSB este a 1

  getBytes ( SI4735_ADDR,2);
      __delay_cycles(1000);
  
  return (response[0] & 0x01);
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void si4735_getREV( void ){
  
	//FW = Firmware and it is a 2 character array
	//CMP = Component Revision and it is a 2 character array
	//REV = Chip Revision and it is a single character

	sprintf(command, "%c", 0x10);
	
	sendBytes(SI4735_ADDR,2);

	getBytes(SI4735_ADDR,9);
	
        sprintf(temp, "FW: %c%c", response[2],response[3]);
        lcd_print("Si4735",0,0);
        
        sprintf(temp, "CMP: %c%c", response[6],response[7]);
        lcd_print(temp,0,2);

        sprintf(temp, "REV: %c", response[8]);
        lcd_print(temp,0,3);
        
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pantalla(void) {
  
  char STBLEND, MULT, FREQOFF;
  
            switch(_modo){
        case FM:
            lcd_print("Modo FM      ",0,0);
            lcd_print("Freq. x10 kHz",0,3);
            sprintf ( command , "%c%c", 0x22, 0x00);
            sendBytes(SI4735_ADDR,2);   
            break;
        case AM:
        case SW:
        case LW:
            lcd_print("Modo AM/LW/SW",0,0);
            lcd_print("Freq (kHz)   ",0,3);
            sprintf ( command , "%c%c", 0x42, 0x00);
            sendBytes(SI4735_ADDR,2);
            break;
        default:
            return;
        }
        
            getBytes(SI4735_ADDR,8);

            sprintf ( temp , "%u", response[4]);
            lcd_print("RSSI:",0,2);
            lcd_print(temp,30,2);
            
            sprintf ( temp , "%u", response[5]);
            lcd_print("SNR:",48,2);
            lcd_print(temp,72,2);

        	if(_modo==FM){
        		STBLEND=response[3]&63;
        		MULT=response[6];
        		FREQOFF=response[7];
        	}
        	else{
        		STBLEND=0;
        		MULT=0;
        		FREQOFF=0;
        	}



            freq=(unsigned int) response[2]<<8 | ( unsigned int) response[3];

            print_int(freq,4);
        
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void si4735_inicia() {
      
         P1OUT |= BIT4;
        
        __delay_cycles(200000);                     //Just a start up delay
        
            switch(_modo){
        case FM:
            sprintf(command, "%c%c%c", 0x01, 0x10, 0x05);
            break;
        case AM:
        case SW:
        case LW:
            sprintf(command, "%c%c%c", 0x01, 0x11, 0x05);
            break;
        default:
            return;
        }
        
        sendBytes(SI4735_ADDR,3);
        
        
        sprintf(command , "%c%c%c%c%c%c", 0x12, 0x00, 0x02, 0x01, 0x80, 0x00);
        sendBytes(SI4735_ADDR,6);
        
        
        si4735_sintoniza();
        
        sprintf ( command , "%c%c%c%c%c%c", 0x12, 0x00, 0x40, 0x00, 0x00, 0x3F);
        sendBytes(SI4735_ADDR,6);
        
        
        sprintf ( command , "%c%c%c%c%c%c", 0x12, 0x00, 0x40, 0x01, 0x00, 0x00);
        sendBytes(SI4735_ADDR,6);
        
        sprintf ( command , "%c%c%c%c%c", 0x12, 0x00, 0x11, 0x05, 0x00);  // fuerza estéreo
        sendBytes(SI4735_ADDR,5);

        sprintf ( command , "%c%c%c%c%c%c", 0x12, 0x00, 0x15, 0x02, 0xAA, 0x01);  // Activa RDS
        sendBytes(SI4735_ADDR,6);

        switch(_modo){
          
        case SW:
                //Set the lower band limit for Short Wave Radio to 2300 kHz
                sprintf(command, "%c%c%c%c%c%c", 0x12, 0x00, 0x34, 0x00, 0x08, 0xFC);
                sendBytes(SI4735_ADDR, 6);
                
                //Set the upper band limit for Short Wave Radio to 23000kHz
                sprintf(command, "%c%c%c%c%c%c", 0x12, 0x00, 0x34, 0x01, 0x59, 0xD8);
                sendBytes(SI4735_ADDR, 6);            
                
                break;
        case LW:
                //Set the lower band limit for Long Wave Radio to 152 kHz
                sprintf(command, "%c%c%c%c%c%c", 0x12, 0x00, 0x34, 0x00, 0x00, 0x99);
                sendBytes(SI4735_ADDR, 6);
                
                //Set the upper band limit for Long Wave Radio to 279 kHz
                sprintf(command, "%c%c%c%c%c%c", 0x12, 0x00, 0x34, 0x01, 0x01, 0x17);
                sendBytes(SI4735_ADDR, 6);
                            
                break;
        default:
                break;
          
        }

        lcd_print("OK!",0,3);

        __delay_cycles(2000000); 
        
        si4735_getREV();
          
        __delay_cycles(2000000); 
        
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void si4735_sintoniza() {
 
        char freqH;
        char freqL;
        
        freqH= freq >> 8;
        freqL= freq & 0xFF;
        
            switch(_modo){
              
        case FM:
            sprintf(command, "%c%c%c%c", 0x20, 0x00, freqH, freqL);
            sendBytes(SI4735_ADDR,4);
            break;

        case AM:
        case SW:
        case LW:
            sprintf(command, "%c%c%c%c%c%c", 0x40, 0x00, freqH, freqL, 0x00, 0x00);
            sendBytes(SI4735_ADDR,6);
            
            break;
        default:
            break;
                }
                  
            

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void si4735_rds ( void ) {
  
  char status,type,version;
  char i,pty;
  int pi;
  
        sprintf(command, "%c%c", 0x24, 0x00);
        sendBytes(SI4735_ADDR,2);
        version = response[6] & 16;
        getBytes(SI4735_ADDR,13);
         
		//response[4] = RDSA high BLOCK1
		//response[5] = RDSA low
		//response[6] = RDSB high BLOCK2
		//response[7] = RDSB low
		//response[8] = RDSC high BLOCK3
		//response[9] = RDSC low
		//response[10] = RDSD high BLOCK4
		//response[11] = RDSD low


        pty= ((response[6]&3) << 3) | ((response[7] >> 5)&7);
        pi= (int) response[4] <<8 | response[5];
        
        type = (response[6]>>4) & 15;
  
        if (type==0) {
          
				  char addr = response[7] & 3;

				  if (addr >= 0 && addr<= 3) {
					if (response[10] != '\0')
						_ps[addr*2] = response[10];
					if (response[11] != '\0')
						_ps[addr*2+1] = response[11];
					_ps_rdy=(addr==3);
				}

		}

        else if (type == 2) {
        		// Get their address
        		char addressRT = response[7] & 15; // Get rightmost 4 bits
        		char ab = response[7] & 16;
         		char cr = 0; //indicates that a carriage return was received
        		char len = 64;

        		if (version == 0) {
        			if (addressRT >= 0 && addressRT <= 15) {
        				if (response[8] != 0x0D )
        					_disp[addressRT*4] = response[8];
        				else{
        					len=addressRT*4;
        					cr=1;
        				}
        				if (response[9] != 0x0D)
        					_disp[addressRT*4+1] = response[9];
        				else{
        					len=addressRT*4+1;
        					cr=1;
        				}
        				if (response[10] != 0x0D)
        					_disp[addressRT*4+2] = response[10];
        				else{
        					len=addressRT*4+2;
        					cr=1;
        				}
        				if (response[11] != 0x0D)
        					_disp[addressRT*4+3] = response[11];
        				else{
        					len=addressRT*4+3;
        					cr=1;
        				}
        			}
        		} else {
        			if (addressRT >= 0 && addressRT <= 7) {
        				if (response[10] != '\0')
        					_disp[addressRT*2] = response[10];
        				if (response[11] != '\0')
        					_disp[addressRT*2+1] = response[11];
        			}
        		}
        		if(cr){
        			for (i=len; i<64; i++)
        				_disp[i] = ' ';
        		}
        		if (ab != _ab) {
        			for (i=0; i<64; i++)
        				_disp[i] = ' ';
        			_disp[64] = '\0';
        			_newRadioText=1;
        		}
        		else{
        			_newRadioText=0;
        		}
        		_ab = ab;
        	}
} 


void si4735_clearRDS(void) {
	char i;
	for(i=0; i<64; i++)	_disp[i] ='\0';
	for(i=0; i<8; i++) 	_ps[i]	 ='\0';
	for(i=0; i<16; i++) _pty[i]	 ='\0';
	for(i=0; i<4; i++) 	_csign[i]='\0';		
}

void si4735_radiotext(void) {

	char i;

	lcd_pos(0, 0);
	lcd_print("PS: ",0, 0);


    if (_ps_rdy) {

    	for ( i=0 ; i<8 ; i++)
    		lcd_character(_ps[i],0,0);
    		_ps_rdy=0;
		}

	if (_scroll_j<50)
		_scroll_j++;
		else
			_scroll_j=0;

	lcd_pos(0, 1);
	for ( i=_scroll_j ; i<_scroll_j+14 ; i++)
		lcd_character(_disp[i],0,0);


}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR_HOOK(void)
{
	/* Port 1 Interrupt Handler */
  
        refresca=1;
        
	__delay_cycles(100000);

	P1IFG &= ~BIT3; // Clear P1.3 IFG
        
        __bic_SR_register_on_exit(LPM0_bits);

	/* No change in operating mode on exit */
}

/*
 *  ======== USCI A0/B0 TX Interrupt Handler Generation ========
 */
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
  
    if (IFG2 & UCB0TXIFG) 
    {
		/* USCI_B0 Transmit Interrupt Handler */
              if (TXByteCtr)                          // Check TX byte counter
              {
                UCB0TXBUF = *PTxData++;               // Load TX buffer
                TXByteCtr--;                          // Decrement TX byte counter
              }
              else
              {
                UCB0CTL1 |= UCTXSTP;                  // I2C stop condition
                IFG2 &= ~UCB0TXIFG;                   // Clear USCI_B0 TX int flag
                /* Enter active mode on exit */
                __bic_SR_register_on_exit(LPM0_bits);
              }  


    }
    else 
    {
              /* USCI_B0 Receive Interrupt Handler */
              RXByteCtr--;                            // Decrement RX byte counter
              if (RXByteCtr)
              {
                //P1OUT ^= 0x10;
                *PRxData++ = UCB0RXBUF;               // Move RX data to address PRxData
                if (RXByteCtr == 1)                   // Only one byte left?
                {
                  UCB0CTL1 |= UCTXSTP;                // Generate I2C stop condition
                }
              }
              else
              {
                 *PRxData = UCB0RXBUF;                 // Move final RX data to PRxData
                 /* Enter active mode on exit */
                 
                 __bic_SR_register_on_exit(LPM0_bits);
              }
              



    }
}

/*
 *  ======== Timer0_A3 Interrupt Service Routine ======== 
 */

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{

	/* Capture Compare Register 0 ISR Hook Function Name */

		//refresca=1;

        P1OUT ^= BIT0;	

	/* No change in operating mode on exit */
}


