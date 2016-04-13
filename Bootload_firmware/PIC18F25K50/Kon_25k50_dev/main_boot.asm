; 
; Author: Bradley A. Minch
; Organization: Franklin W. Olin College of Engineering
; Revision History: 
;     01/19/2006 - Added wait for initial SE0 condition to clear at the end
;                  of InitUSB.
;     01/13/2006 - Fixed problem with DATA OUT transfers (only worked properly
;                  with class requests) in ProcessSetupToken.  Added code to
;                  disable all EPs except EP0 on a valid SET_CONFIGURATION
;                  request.  Changed code to use BSTALL instead of EPSTALL for
;                  Request Error on EP0.  Changed CLEAR_FEATURE, SET_FEATURE
;                  and GET_STATUS requests to use BSTALL instead of EPSTALL.
;                  Changed over from the deprecated __CONFIG assembler directive 
;                  to config for setting the configuration bits.  Eliminated the
;                  initial for loop from the start of the main section.
;     06/22/2005 - Added code to disable all endpoints on USRTIF and to mask
;                  bits 0, 1, and 7 of USTAT on TRNIF in ServiceUSB.
;     04/21/2005 - Initial public release.
;
;     11/07/2010 - The code was adapted to the low pin count PIC18f14k50
;                  by Peter Jakab, see http://jap.hu/electronic/pic18-usb.html
;
;     12/14/2016 - The code was adapted to act as a HID device based 
;		   bootloader by Konstantin Avdashchenko
;
; ============================================================================
; 
; Peripheral Description:
; 
; This peripheral enumerates as a HID device. It then allows for rewriting the
; firmware located at 0x1000-onwards. The method for sending data to the device
; is based an a slighly modified halfkay protocol where there are 4 bytes 
; assigned to addressing and control rather than only 2.
; 
; ============================================================================
;
; Software Licence Agreement:
; 
; THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION.  NO WARRANTIES, WHETHER 
; EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED 
; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY 
; TO THIS SOFTWARE. THE AUTHOR SHALL NOT, UNDER ANY CIRCUMSTANCES, BE LIABLE 
; FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
#include <p18f25k50.inc>
#include "usb_defs.inc"
#include "ENGR2210.inc"

; CONFIG1L
  CONFIG  PLLSEL = PLL3X        ; PLL Selection (3x clock multiplier)
  CONFIG  CFGPLLEN = ON         ; PLL Enable Configuration bit (PLL Enabled)
  CONFIG  CPUDIV = NOCLKDIV     ; CPU System Clock Postscaler (CPU uses system clock (no divide))
  CONFIG  LS48MHZ = SYS48X8     ; Low Speed USB mode with 48 MHz system clock (System clock at 48 MHz, USB clock divider is set to 8)

; CONFIG1H
  CONFIG  FOSC = INTOSCIO       ; Oscillator Selection (Internal oscillator)
  CONFIG  PCLKEN = ON           ; Primary Oscillator Shutdown (Primary oscillator enabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor (Fail-Safe Clock Monitor disabled)
  CONFIG  IESO = OFF            ; Internal/External Oscillator Switchover (Oscillator Switchover mode disabled)

; CONFIG2L
  CONFIG  nPWRTEN = ON          ; Power-up Timer Enable (Power up timer enabled)
  CONFIG  BOREN = OFF           ; Brown-out Reset Enable (BOR disabled in hardware (SBOREN is ignored))
  CONFIG  BORV = 285            ; Brown-out Reset Voltage (BOR set to 2.85V nominal)
  CONFIG  nLPBOR = OFF          ; Low-Power Brown-out Reset (Low-Power Brown-out Reset disabled)

; CONFIG2H
  CONFIG  WDTEN = SWON            ; Watchdog Timer Enable bits (WDT disabled in hardware (SWDTEN ignored))
  CONFIG  WDTPS = 1             ; Watchdog Timer Postscaler (1:1)

; CONFIG3H
  CONFIG  CCP2MX = RB3          ; CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
  CONFIG  PBADEN = OFF          ; PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
  CONFIG  T3CMX = RB5           ; Timer3 Clock Input MUX bit (T3CKI function is on RB5)
  CONFIG  SDOMX = RB3           ; SDO Output MUX bit (SDO function is on RB3)
  CONFIG  MCLRE = ON            ; Master Clear Reset Pin Enable (RE3 input pin enabled; external MCLR disabled)

; CONFIG4L
  CONFIG  STVREN = ON           ; Stack Full/Underflow Reset (Stack full/underflow will cause Reset)
  CONFIG  LVP = ON              ; Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
  CONFIG  XINST = ON            ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled)

; CONFIG5L
  CONFIG  CP0 = OFF             ; Block 0 Code Protect (Block 0 is not code-protected)
  CONFIG  CP1 = OFF             ; Block 1 Code Protect (Block 1 is not code-protected)
  CONFIG  CP2 = OFF             ; Block 2 Code Protect (Block 2 is not code-protected)
  CONFIG  CP3 = OFF             ; Block 3 Code Protect (Block 3 is not code-protected)

; CONFIG5H
  CONFIG  CPB = OFF             ; Boot Block Code Protect (Boot block is not code-protected)
  CONFIG  CPD = OFF             ; Data EEPROM Code Protect (Data EEPROM is not code-protected)

; CONFIG6L
  CONFIG  WRT0 = OFF            ; Block 0 Write Protect (Block 0 (0800-1FFFh) is not write-protected)
  CONFIG  WRT1 = OFF            ; Block 1 Write Protect (Block 1 (2000-3FFFh) is not write-protected)
  CONFIG  WRT2 = OFF            ; Block 2 Write Protect (Block 2 (04000-5FFFh) is not write-protected)
  CONFIG  WRT3 = OFF            ; Block 3 Write Protect (Block 3 (06000-7FFFh) is not write-protected)

; CONFIG6H
  CONFIG  WRTC = OFF            ; Configuration Registers Write Protect (Configuration registers (300000-3000FFh) are not write-protected)
  CONFIG  WRTB = ON             ; Boot Block Write Protect (Boot block (0000-7FFh) is not write-protected)
  CONFIG  WRTD = OFF            ; Data EEPROM Write Protect (Data EEPROM is not write-protected)

; CONFIG7L
  CONFIG  EBTR0 = OFF           ; Block 0 Table Read Protect (Block 0 is not protected from table reads executed in other blocks)
  CONFIG  EBTR1 = OFF           ; Block 1 Table Read Protect (Block 1 is not protected from table reads executed in other blocks)
  CONFIG  EBTR2 = OFF           ; Block 2 Table Read Protect (Block 2 is not protected from table reads executed in other blocks)
  CONFIG  EBTR3 = OFF           ; Block 3 Table Read Protect (Block 3 is not protected from table reads executed in other blocks)

; CONFIG7H
  CONFIG  EBTRB = OFF           ; Boot Block Table Read Protect (Boot block is protected from table reads executed in other blocks)


#define	SHOW_ENUM_STATUS

#define	USER_APPLICATION     0x1000

bank0		udata
USB_buffer_desc	res		4
USB_buffer_data	res     8
USB_error_flags	res 	1
USB_curr_config	res		1
USB_device_status	res	1
USB_dev_req	res			1
USB_address_pending	res	1
USB_desc_ptr	res		1
USB_bytes_left	res		1
USB_loop_index	res		1
USB_packet_length	res	1
USB_USTAT	res			1
USB_USWSTAT	res			1
PROGRAM_IT  res         1
COUNTER_L		res		1
COUNTER_H		res		1
COUNTER         res     1

bank1       udata
PROG_add_lo   res     1
PROG_add_hi   res     1
PROG_add_lup  res     1
PROG_add_hup  res     1
PROGRAM_data  res     64

STARTUP		code		0x0000
			goto		Main					; Reset vector
			nop
			nop
			goto		0x1008 ;Resides at 0x0008 (hardware high priority interrupt vector), and causes PC to jump to 0x1008 upon a high priority interrupt event
			nop
			nop
			nop
			nop
			nop
			nop
			goto		0x1018 ;Resides at 0x0018 (hardware low priority interrupt vector), and causes PC to jump to 0x1018 upon a low priority interrupt event


USBSTUFF	code
Descriptor
			movlw		upper Descriptor_begin
			movwf		TBLPTRU, ACCESS
			movlw		high Descriptor_begin
			movwf		TBLPTRH, ACCESS
			movlw		low Descriptor_begin
			banksel		USB_desc_ptr
			addwf		USB_desc_ptr, W, BANKED
			ifset STATUS, C, ACCESS
				incf		TBLPTRH, F, ACCESS
				ifset STATUS, Z, ACCESS
					incf		TBLPTRU, F, ACCESS
				endi
			endi
			movwf		TBLPTRL, ACCESS
			tblrd*
			movf		TABLAT, W
			return

Descriptor_begin
Device
			db			0x12, DEVICE		; bLength, bDescriptorType
			db			0x00, 0x02			; bcdUSB (low byte), bcdUSB (high byte)
			db			0x00, 0x02			; bDeviceClass, bDeviceSubClass
			db			0x00, MAX_PACKET_SIZE	; bDeviceProtocol, bMaxPacketSize
			db			0x09, 0x12			; idVendor (low byte), idVendor (high byte)
			db			0x07, 0xB0			; idProduct (low byte), idProduct (high byte)
			db			0x01, 0x00			; bcdDevice (low byte), bcdDevice (high byte)
			db			0x01, 0x02			; iManufacturer, iProduct
			db			0x00, NUM_CONFIGURATIONS	; iSerialNumber (none), bNumConfigurations

Configuration1
			db			0x09, CONFIGURATION	; bLength, bDescriptorType
			db			String0-Configuration1, 0x00			; wTotalLength (low byte), wTotalLength (high byte)
			db			NUM_INTERFACES, 0x01	; bNumInterfaces, bConfigurationValue
			db			0x00, 0xA0			; iConfiguration (none), bmAttributes
			db			0x32, 0x09			; bMaxPower (100 mA), bLength (Interface1 descriptor starts here)
			db			INTERFACE, 0x00		; bDescriptorType, bInterfaceNumber
			db			0x00, 0x01			; bAlternateSetting, bNumEndpoints (excluding EP0)
			db			0x03, 0x00			; bInterfaceClass (HID), bInterfaceSubClass
			db			0x00, 0x00			; bInterfaceProtocol (vendor specific protocol used), iInterface (none)
;HID_interface_descriptor                    ; HID 1.11 spec section 6.2.1
            db          0x09, HID                              ; bLength ; bDescriptorType
            db          0x11, 0x01                             ; bcdHID
            db          0x00, 0x02                             ; bCountryCode; bNumDescriptors
            db          0x22, 0x1C                             ; bDescriptorType; wDescriptorLength
            db          0x00, 0x07                             ;?? ; bLength
            db          ENDPOINT, 0x01 | 0x80                      ; bDescriptorType ; bEndpointAddress
            db          0x03, RAWHID_TX_SIZE                   ; bmAttributes (0x03=intr); wMaxPacketSize
            db          0x00, RAWHID_TX_INTERVAL              ;?? ; bInterval
;endpoint_descriptor1                         ; USB spec 9.6.6, page 269-271, Table 9-13
;            db          0x07, ENDPOINT                             ; bLength; bDescriptorType
;            db          0x02, 0x03                             ; bEndpointAddress; bmAttributes (0x03=intr)
;            db          RAWHID_RX_SIZE, 0x00                   ; wMaxPacketSize
;            db          RAWHID_RX_INTERVAL                     ; bInterval

String0
			db			String1-String0, STRING	; bLength, bDescriptorType
			db			0x09, 0x04			; wLANGID[0] (low byte), wLANGID[0] (high byte)
String1
			db			String2-String1, STRING	; bLength, bDescriptorType
			db			'G', 0x00			; bString
			db			'l', 0x00
			db			'o', 0x00
			db			'b', 0x00
			db			'a', 0x00
			db			'l', 0x00
			db			' ', 0x00
			db			'B', 0x00
			db			'o', 0x00
			db			'o', 0x00
			db			't', 0x00
			db			'l', 0x00
			db			'o', 0x00
			db			'a', 0x00
			db			'd', 0x00
			db			'e', 0x00
			db			'r', 0x00
			db			' ', 0x00
			db			'f', 0x00
			db			'o', 0x00
			db			'r', 0x00
			db			' ', 0x00
			db			'Y', 0x00
			db			'o', 0x00
			db			'u', 0x00
			db			'!', 0x00
String2
			db			rawhid_report_desc-String2, STRING	; bLength, bDescriptorType
			db			'K', 0x00			; bString
			db			'o', 0x00
			db			'n', 0x00
			db			's', 0x00
			db			'g', 0x00
			db			'n', 0x00
			db			'\'', 0x00
			db			's', 0x00
			db			' ', 0x00
			db			'P', 0x00
			db			'I', 0x00
			db			'C', 0x00
			db			'1', 0x00
			db			'8', 0x00
			db			'F', 0x00
			db			'2', 0x00
			db			'5', 0x00
			db			'K', 0x00
			db			'5', 0x00
			db			'0', 0x00
			db			' ', 0x00
			db			'U', 0x00
			db			'S', 0x00
			db			'B', 0x00
			db			' ', 0x00
			db			'B', 0x00
			db			'o', 0x00
			db			'o', 0x00
			db			't', 0x00
			db			'l', 0x00
			db			'o', 0x00
			db			'a', 0x00
			db			'd', 0x00
			db			'e', 0x00
			db			'r', 0x00

rawhid_report_desc
;            db          0x06, 0xAB, 0xFF                       ; Usage Page Low, HIGH
;            db          0x0A, 0x00, 0x02                       ; Usage Low,HIGH
;            db          0xA1, 0x01                             ; Collection 0x01
;            db          0x75, 0x08                             ; report size = 8 bits
;            db          0x15, 0x00                             ; logical minimum = 0
;            db          0x26, 0xFF, 0x00                       ; logical maximum = 255
;            db          0x95, RAWHID_TX_SIZE                   ; report count
;            db          0x09, 0x01                             ; usage
;            db          0x81, 0x02                             ; Input (array)
;            db          0x95, RAWHID_RX_SIZE                   ; report count
;            db          0x09, 0x02                             ; usage
;            db          0x91, 0x02                             ; Output (array)
;            db          0xC0                                   ; end collection
            db          0x06, 0xAB                       
            db          0xFF, 0x0A                      
            db          0x00, 0x02
            db          0xA1, 0x01                             
            db          0x75, 0x08                             
            db          0x15, 0x00                             
            db          0x26, 0xFF
            db          0x00, 0x95
            db          RAWHID_TX_SIZE, 0x09
            db          0x01, 0x81
            db          0x02, 0x95
            db          RAWHID_RX_SIZE, 0x09
            db          0x02, 0x91
            db          0x02, 0xC0
Descriptor_end

InitUSB
			clrf		UIE, ACCESS				; mask all USB interrupts
			clrf		UIR, ACCESS				; clear all USB interrupt flags
			movlw		0x14
			movwf		UCFG, ACCESS			; configure USB for full-speed transfers and to use the on-chip transciever and pull-up resistor
            repeat
            movlw		0x08
			movwf		UCON, ACCESS			; enable the USB module and its supporting circuitry
            untilset    UCON,USBEN,ACCESS
            banksel		USB_curr_config
			clrf		USB_curr_config, BANKED
			clrf		USB_USWSTAT, BANKED		; default to powered state
			movlw		0x01
			movwf		USB_device_status, BANKED
			movlw		NO_REQUEST
			movwf		USB_dev_req, BANKED		; No device requests in process
			repeat								; do nothing...
			untilclr UCON, SE0, ACCESS			; ...until initial SE0 condition clears
#ifdef SHOW_ENUM_STATUS
            bcf         PORTA,RA6,ACCESS        ; Red
            bcf         PORTC,RC0,ACCESS        ; Green
            bcf         PORTC,RC1,ACCESS        ; Blue
#endif
			return

ServiceUSB
			select
				caseset	UIR, UERRIF, ACCESS
					banksel		UEIR
					clrf		UEIR, BANKED
					break
				caseset UIR, SOFIF, ACCESS
					bcf			UIR, SOFIF, ACCESS
					break
				caseset	UIR, IDLEIF, ACCESS
					bcf			UIR, IDLEIF, ACCESS
					bsf			UCON, SUSPND, ACCESS
					break
				caseset UIR, ACTVIF, ACCESS
					bcf			UIR, ACTVIF, ACCESS
					bcf			UCON, SUSPND, ACCESS
;#ifdef SHOW_ENUM_STATUS
;					banksel		USB_USWSTAT
;					movf		USB_USWSTAT, W, BANKED
;					select
;						case POWERED_STATE
;							movlw	0x02
;							break
;						case DEFAULT_STATE
;							movlw	0x02
;							break
;						case ADDRESS_STATE
;							movlw	0x00
;							break
;						case CONFIG_STATE
;							movlw	0x00
;					ends
;					iorwf		PORTC, F, ACCESS
;#endif
					break
				caseset	UIR, STALLIF, ACCESS
					bcf			UIR, STALLIF, ACCESS
					break
				caseset	UIR, URSTIF, ACCESS
					banksel		USB_curr_config
					clrf		USB_curr_config, BANKED
					bcf			UIR, TRNIF, ACCESS		; clear TRNIF four times to clear out the USTAT FIFO
					bcf 		UIR, TRNIF, ACCESS
					bcf			UIR, TRNIF, ACCESS
					bcf			UIR, TRNIF, ACCESS
					banksel UEP0
					clrf		UEP0, BANKED			; clear all EP control registers to disable all endpoints
					clrf		UEP1, BANKED
					clrf		UEP2, BANKED
					clrf		UEP3, BANKED
					clrf		UEP4, BANKED
					clrf		UEP5, BANKED
					clrf		UEP6, BANKED
					clrf		UEP7, BANKED
					banksel		BD0OBC
					movlw		MAX_PACKET_SIZE
					movwf		BD0OBC, BANKED
					movlw		low USB_Buffer			; EP0 OUT gets a buffer...
					movwf		BD0OAL, BANKED
					movlw		high USB_Buffer
					movwf		BD0OAH, BANKED			; ...set up its address
					movlw		0x88					; set UOWN bit (USB can write)
					movwf		BD0OST, BANKED
					movlw		low (USB_Buffer+USB_BUFFER_SIZE)	; EP0 IN gets a buffer... ;kon
					movwf		BD0IAL, BANKED
					movlw		high (USB_Buffer+USB_BUFFER_SIZE)
					movwf		BD0IAH, BANKED			; ...set up its address
					movlw		0x08					; clear UOWN bit (MCU can write)
					movwf		BD0IST, BANKED
					banksel		UADDR
					clrf		UADDR, BANKED			; set USB Address to 0
					clrf		UIR, ACCESS				; clear all the USB interrupt flags
					movlw		ENDPT_CONTROL
					movwf		UEP0, BANKED			; EP0 is a control pipe and requires an ACK
					movlw		0xFF					; enable all error interrupts
					movwf		UEIE, BANKED
					banksel		USB_USWSTAT
					movlw		DEFAULT_STATE
					movwf		USB_USWSTAT, BANKED
					movlw		0x01
					movwf		USB_device_status, BANKED	; self powered, remote wakeup disabled
#ifdef SHOW_ENUM_STATUS
					;movlw		0xE0
					;andwf		PORTC, F, ACCESS
					bsf 		PORTC, 1, ACCESS		; set bit 1 of PORTB to indicate Powered state
#endif
					break
				caseset	UIR, TRNIF, ACCESS
					movlw		high BD0OST
					movwf		FSR0H, ACCESS
					movf		USTAT, W, ACCESS
					andlw		0x7C					; mask out bits 0, 1, and 7 of USTAT
					movwf		FSR0L, ACCESS
					banksel		USB_buffer_desc
					movf		POSTINC0, W
					movwf		USB_buffer_desc, BANKED
					movf		POSTINC0, W
					movwf		USB_buffer_desc+1, BANKED
					movf		POSTINC0, W
					movwf		USB_buffer_desc+2, BANKED
					movf		POSTINC0, W
					movwf		USB_buffer_desc+3, BANKED
					movf		USTAT, W, ACCESS
					movwf		USB_USTAT, BANKED		; save the USB status register
					bcf			UIR, TRNIF, ACCESS		; clear TRNIF interrupt flag
;#ifdef SHOW_ENUM_STATUS
;					andlw		0x18					; extract EP bits
;					select
;						case EP0
;							movlw		0x20
;							break
;						case EP1
;							movlw		0x40
;							break
;						case EP2
;							movlw		0x80
;							break
;					ends
;					xorwf		PORTC, F, ACCESS		; toggle bit 5, 6, or 7 of PORTB to reflect EP activity
;#endif
					clrf		USB_error_flags, BANKED	; clear USB error flags
					movf		USB_buffer_desc, W, BANKED
					andlw		0x3C					; extract PID bits
					select
						case TOKEN_SETUP
							call		ProcessSetupToken
							break
						case TOKEN_IN
							call		ProcessInToken
							break
						case TOKEN_OUT
							btg         PORTC,1,ACCESS
							call		ProcessOutToken
							break
						default
					ends
					banksel USB_error_flags
					ifset USB_error_flags, 0, BANKED	; if there was a Request Error...
						banksel		BD0OBC
						movlw		MAX_PACKET_SIZE
						movwf		BD0OBC				; ...get ready to receive the next Setup token...
						movlw		0x84
						movwf		BD0IST
						movwf		BD0OST				; ...and issue a protocol stall on EP0
					endi
					break
			ends
			return

ProcessSetupToken
			banksel		USB_buffer_data
			movf		USB_buffer_desc+ADDRESSH, W, BANKED
			movwf		FSR0H, ACCESS
			movf		USB_buffer_desc+ADDRESSL, W, BANKED
			movwf		FSR0L, ACCESS
			movf		POSTINC0, W
			movwf		USB_buffer_data, BANKED
			movf		POSTINC0, W
			movwf		USB_buffer_data+1, BANKED
			movf		POSTINC0, W
			movwf		USB_buffer_data+2, BANKED
			movf		POSTINC0, W
			movwf		USB_buffer_data+3, BANKED
			movf		POSTINC0, W
			movwf		USB_buffer_data+4, BANKED
			movf		POSTINC0, W
			movwf		USB_buffer_data+5, BANKED
			movf		POSTINC0, W
			movwf		USB_buffer_data+6, BANKED
			movf		POSTINC0, W
			movwf		USB_buffer_data+7, BANKED
			banksel		BD0OBC
;kon
;            bsf         PORTC,0,ACCESS
;            ifl         USB_buffer_desc+BYTECOUNT,>=,9
;                bcf         PORTC,0,ACCESS
;                clrf        COUNTER_L
;                banksel     PROG_add_lo
;                movlw       low PROG_add_lo
;                movwf       FSR1L
;                movlw       high PROG_add_lo
;                movwf       FSR1H
;                banksel     COUNTER_L
;                for         COUNTER_L,0,68
;                    movf		POSTINC0, W
;                    movwf       POSTINC1
;                next COUNTER_L
;            endi

			movlw		MAX_PACKET_SIZE
			movwf		BD0OBC, BANKED					; reset the byte count
			movwf		BD0IST, BANKED					; return the in buffer to us (dequeue any pending requests)
			banksel		USB_buffer_data+bmRequestType
			ifclr USB_buffer_data+bmRequestType, 7, BANKED
				ifl USB_buffer_data+wLength, !=, 0
            	orif USB_buffer_data+wLengthHigh, !=, 0
					movlw		0xC8
				otherwise
					movlw		0x88
				endi
			otherwise
				movlw		0x88
			endi
			banksel		BD0OST
			movwf		BD0OST, BANKED					; set EP0 OUT UOWN back to USB and DATA0/DATA1 packet according to request type
			bcf			UCON, PKTDIS, ACCESS			; assuming there is nothing to dequeue, clear the packet disable bit
			banksel		USB_dev_req
			movlw		NO_REQUEST
			movwf		USB_dev_req, BANKED				; clear the device request in process
			movf		USB_buffer_data+bmRequestType, W, BANKED
			andlw		0x60							; extract request type bits
			select
				case STANDARD
					call		StandardRequests
					break
				case CLASS
					call		ClassRequests
					break
				case VENDOR
					call		VendorRequests
					break
				default
					bsf			USB_error_flags, 0, BANKED	; set Request Error flag
			ends
			return

StandardRequests
			movf		USB_buffer_data+bRequest, W, BANKED
			select
				case GET_STATUS
					movf		USB_buffer_data+bmRequestType, W, BANKED
					andlw		0x1F					; extract request recipient bits
					select
						case RECIPIENT_DEVICE
							banksel		BD0IAH
							movf		BD0IAH, W, BANKED
							movwf		FSR0H, ACCESS
							movf		BD0IAL, W, BANKED				; get buffer pointer
							movwf		FSR0L, ACCESS
							banksel		USB_device_status
							movf		USB_device_status, W, BANKED	; copy device status byte to EP0 buffer
							movwf		POSTINC0
							clrf		INDF0
							banksel		BD0IBC
							movlw		0x02
							movwf		BD0IBC, BANKED					; set byte count to 2
							movlw		0xC8
							movwf		BD0IST, BANKED					; send packet as DATA1, set UOWN bit
							break
						case RECIPIENT_INTERFACE
							movf		USB_USWSTAT, W, BANKED
							select
								case ADDRESS_STATE
									bsf			USB_error_flags, 0, BANKED		; set Request Error flag
									break
								case CONFIG_STATE
									ifl USB_buffer_data+wIndex, <, NUM_INTERFACES
										banksel		BD0IAH
										movf		BD0IAH, W, BANKED
										movwf		FSR0H, ACCESS
										movf		BD0IAL, W, BANKED				; get buffer pointer
										movwf		FSR0L, ACCESS
										clrf		POSTINC0
										clrf		INDF0
										movlw		0x02
										movwf		BD0IBC, BANKED					; set byte count to 2
										movlw		0xC8
										movwf		BD0IST, BANKED					; send packet as DATA1, set UOWN bit
									otherwise
										bsf			USB_error_flags, 0, BANKED		; set Request Error flag
									endi
									break
							ends
							break
						case RECIPIENT_ENDPOINT
							movf		USB_USWSTAT, W, BANKED
							select
								case ADDRESS_STATE
									movf		USB_buffer_data+wIndex, W, BANKED	; get EP
									andlw		0x0F								; strip off direction bit
									ifset STATUS, Z, ACCESS							; see if it is EP0
										banksel		BD0IAH
										movf		BD0IAH, W, BANKED				; put EP0 IN buffer pointer...
										movwf		FSR0H, ACCESS
										movf		BD0IAL, W, BANKED
										movwf		FSR0L, ACCESS					; ...into FSR0
										banksel		USB_buffer_data+wIndex
										ifset USB_buffer_data+wIndex, 7, BANKED		; if the specified direction is IN...
											banksel		BD0IST
											movf		BD0IST, W, BANKED
										otherwise
											banksel		BD0OST
											movf		BD0OST, W, BANKED
										endi
										andlw		0x04							; extract the BSTALL bit
										movwf		INDF0
										rrncf		INDF0, F
										rrncf		INDF0, F						; shift BSTALL bit into the lsb position
										clrf		PREINC0
										movlw		0x02
										movwf		BD0IBC, BANKED					; set byte count to 2
										movlw		0xC8
										movwf		BD0IST, BANKED					; send packet as DATA1, set UOWN bit
									otherwise
										bsf			USB_error_flags, 0, BANKED		; set Request Error flag
									endi
									break
								case CONFIG_STATE
									banksel		BD0IAH
									movf		BD0IAH, W, BANKED					; put EP0 IN buffer pointer...
									movwf		FSR0H, ACCESS
									movf		BD0IAL, W, BANKED
									movwf		FSR0L, ACCESS						; ...into FSR0
									movlw		high UEP0							; put UEP0 address...
									movwf		FSR1H, ACCESS
									movlw		low UEP0
									movwf		FSR1L, ACCESS						; ...into FSR1
									movlw		high BD0OST							; put BDndST address...
									movwf		FSR2H, ACCESS
									banksel		USB_buffer_data+wIndex
									movf		USB_buffer_data+wIndex, W, BANKED
									andlw		0x8F								; mask out all but the direction bit and EP number
									movwf		FSR2L, ACCESS
									rlncf		FSR2L, F, ACCESS
									rlncf		FSR2L, F, ACCESS
									rlncf		FSR2L, F, ACCESS					; FSR2L now contains the proper offset into the BD table for the specified EP
									movlw		low BD0OST
									addwf		FSR2L, F, ACCESS					; ...into FSR2
									ifset STATUS, C, ACCESS
										incf		FSR2H, F, ACCESS
									endi
									movf		USB_buffer_data+wIndex, W, BANKED	; get EP and...
									andlw		0x0F								; ...strip off direction bit
									ifset USB_buffer_data+wIndex, 7, BANKED			; if the specified EP direction is IN...
									andifclr PLUSW1, EPINEN, ACCESS					; ...and the specified EP is not enabled for IN transfers...
										bsf			USB_error_flags, 0, BANKED		; ...set Request Error flag
									elsifclr USB_buffer_data+wIndex, 7, BANKED		; otherwise, if the specified EP direction is OUT...
									andifclr PLUSW1, EPOUTEN, ACCESS				; ...and the specified EP is not enabled for OUT transfers...
										bsf			USB_error_flags, 0, BANKED		; ...set Request Error flag
									otherwise
										movf		INDF2, W						; move the contents of the specified BDndST register into WREG
										andlw		0x04							; extract the BSTALL bit
										movwf		INDF0
										rrncf		INDF0, F
										rrncf		INDF0, F						; shift BSTALL bit into the lsb position
										clrf		PREINC0
										banksel		BD0IBC
										movlw		0x02
										movwf		BD0IBC, BANKED					; set byte count to 2
										movlw		0xC8
										movwf		BD0IST, BANKED					; send packet as DATA1, set UOWN bit
									endi
									break
								default
									bsf			USB_error_flags, 0, BANKED	; set Request Error flag
							ends
							break
						default
							bsf			USB_error_flags, 0, BANKED	; set Request Error flag
					ends
					break
				case CLEAR_FEATURE
				case SET_FEATURE
					movf		USB_buffer_data+bmRequestType, W, BANKED
					andlw		0x1F					; extract request recipient bits
					select
						case RECIPIENT_DEVICE
							movf		USB_buffer_data+wValue, W, BANKED
							select
								case DEVICE_REMOTE_WAKEUP
									ifl USB_buffer_data+bRequest, ==, CLEAR_FEATURE
										bcf			USB_device_status, 1, BANKED
									otherwise
										bsf			USB_device_status, 1, BANKED
									endi
									banksel		BD0IBC
									clrf		BD0IBC, BANKED					; set byte count to 0
									movlw		0xC8
									movwf		BD0IST, BANKED					; send packet as DATA1, set UOWN bit
									break
								default
									bsf			USB_error_flags, 0, BANKED		; set Request Error flag
							ends
							break
						case RECIPIENT_ENDPOINT
							movf		USB_USWSTAT, W, BANKED
							select
								case ADDRESS_STATE
									movf		USB_buffer_data+wIndex, W, BANKED	; get EP
									andlw		0x0F								; strip off direction bit
									ifset STATUS, Z, ACCESS							; see if it is EP0
										banksel		BD0IBC
										clrf		BD0IBC, BANKED					; set byte count to 0
										movlw		0xC8
										movwf		BD0IST, BANKED					; send packet as DATA1, set UOWN bit
									otherwise
										bsf			USB_error_flags, 0, BANKED		; set Request Error flag
									endi
									break
								case CONFIG_STATE
									movlw		high UEP0							; put UEP0 address...
									movwf		FSR0H, ACCESS
									movlw		low UEP0
									movwf		FSR0L, ACCESS						; ...into FSR0
									movlw		high BD0OST							; put BD0OST address...
									movwf		FSR1H, ACCESS
									movlw		low BD0OST
									movwf		FSR1L, ACCESS						; ...into FSR1
									movf		USB_buffer_data+wIndex, W, BANKED	; get EP
									andlw		0x0F								; strip off direction bit
									ifclr STATUS, Z, ACCESS							; if it was not EP0...
										addwf		FSR0L, F, ACCESS					; add EP number to FSR0
										ifset		STATUS, C, ACCESS
											incf		FSR0H, F, ACCESS
										endi
										rlncf		USB_buffer_data+wIndex, F, BANKED
										rlncf		USB_buffer_data+wIndex, F, BANKED
										rlncf		USB_buffer_data+wIndex, W, BANKED	; WREG now contains the proper offset into the BD table for the specified EP
										andlw		0x7C								; mask out all but the direction bit and EP number (after three left rotates)
										addwf		FSR1L, F, ACCESS					; add BD table offset to FSR1
										ifset		STATUS, C, ACCESS
											incf		FSR1H, F, ACCESS
										endi
										ifset USB_buffer_data+wIndex, 1, BANKED			; if the specified EP direction (now bit 1) is IN...
											ifset INDF0, EPINEN, ACCESS						; if the specified EP is enabled for IN transfers...
												ifl USB_buffer_data+bRequest, ==, CLEAR_FEATURE
													clrf		INDF1					; clear the stall on the specified EP
												otherwise
													movlw		0x84
													movwf		INDF1					; stall the specified EP
												endi
											otherwise
												bsf			USB_error_flags, 0, BANKED		; set Request Error flag
											endi
										otherwise										; ...otherwise the specified EP direction is OUT, so...
											ifset INDF0, EPOUTEN, ACCESS					; if the specified EP is enabled for OUT transfers...
												ifl USB_buffer_data+bRequest, ==, CLEAR_FEATURE
													movlw		0x88
													movwf		INDF1					; clear the stall on the specified EP
												otherwise
													movlw		0x84
													movwf		INDF1					; stall the specified EP
												endi
											otherwise
												bsf			USB_error_flags, 0, BANKED		; set Request Error flag
											endi
										endi
									endi
									ifclr USB_error_flags, 0, BANKED	; if there was no Request Error...
										banksel		BD0IBC
										clrf		BD0IBC, BANKED			; set byte count to 0
										movlw		0xC8
										movwf		BD0IST, BANKED			; send packet as DATA1, set UOWN bit
									endi
									break
								default
									bsf			USB_error_flags, 0, BANKED	; set Request Error flag
							ends
							break
						default
							bsf			USB_error_flags, 0, BANKED	; set Request Error flag
					ends
					break
				case SET_ADDRESS
					ifset USB_buffer_data+wValue, 7, BANKED		; if new device address is illegal, send Request Error
						bsf			USB_error_flags, 0, BANKED	; set Request Error flag
					otherwise
						movlw		SET_ADDRESS
						movwf		USB_dev_req, BANKED			; processing a SET_ADDRESS request
						movf		USB_buffer_data+wValue, W, BANKED
						movwf		USB_address_pending, BANKED	; save new address
						banksel		BD0IBC
						clrf		BD0IBC, BANKED				; set byte count to 0
						movlw		0xC8
						movwf		BD0IST, BANKED				; send packet as DATA1, set UOWN bit
					endi
					break
				case GET_DESCRIPTOR
					movwf		USB_dev_req, BANKED				; processing a GET_DESCRIPTOR request
					movf		USB_buffer_data+(wValue+1), W, BANKED
					select
						case DEVICE
							movlw		low (Device-Descriptor_begin)
							movwf		USB_desc_ptr, BANKED
							call		Descriptor				; get descriptor length
							movwf		USB_bytes_left, BANKED
							ifl USB_buffer_data+(wLength+1), ==, 0
							andiff USB_buffer_data+wLength, <, USB_bytes_left
								movf		USB_buffer_data+wLength, W, BANKED
								movwf		USB_bytes_left, BANKED
							endi
							call		SendDescriptorPacket
							break
						case CONFIGURATION
							movf		USB_buffer_data+wValue, W, BANKED
							select
								case 0
									movlw		low (Configuration1-Descriptor_begin)
									break
								default
									bsf			USB_error_flags, 0, BANKED	; set Request Error flag
							ends
							ifclr USB_error_flags, 0, BANKED
								addlw		0x02				; add offset for wTotalLength
								movwf		USB_desc_ptr, BANKED
								call		Descriptor			; get total descriptor length
								movwf		USB_bytes_left, BANKED
								movlw		0x02
								subwf		USB_desc_ptr, F, BANKED	; subtract offset for wTotalLength
								ifl USB_buffer_data+(wLength+1), ==, 0
								andiff USB_buffer_data+wLength, <, USB_bytes_left
									movf		USB_buffer_data+wLength, W, BANKED
									movwf		USB_bytes_left, BANKED
								endi
								call		SendDescriptorPacket
							endi
							break
;						case QUALIFIER
;							bsf         PORTA,RA6,ACCESS        ; Red
;							bcf         PORTC,RC0,ACCESS        ; Green
;							bcf         PORTC,RC1,ACCESS        ; Blue
;							banksel	    USB_error_flags
;							bsf	    USB_error_flags, 0, BANKED	; set Request Error flag
;							break	
						case REPORT
							movf		USB_buffer_data+wValue, W, BANKED
							select
								case 0
									movlw		low (rawhid_report_desc-Descriptor_begin)
									break
								default
									bsf			USB_error_flags, 0, BANKED	; set Request Error flag
							ends
							ifclr USB_error_flags, 0, BANKED
								movwf       USB_desc_ptr, BANKED
								movlw		HID_REPORT_SIZE			; get total descriptor length
								movwf		USB_bytes_left, BANKED
								ifl USB_buffer_data+(wLength+1), ==, 0
								andiff USB_buffer_data+wLength, <, USB_bytes_left
									movf		USB_buffer_data+wLength, W, BANKED
									movwf		USB_bytes_left, BANKED
								endi
								call		SendDescriptorPacket
							endi
							break
						case STRING
							movf		USB_buffer_data+wValue, W, BANKED
							select
								case 0
									movlw		low (String0-Descriptor_begin)
									break
								case 1
									movlw		low (String1-Descriptor_begin)
									break
								case 2
									movlw		low (String2-Descriptor_begin)
									break
								default
									bsf			USB_error_flags, 0, BANKED	; Set Request Error flag
							ends
							ifclr USB_error_flags, 0, BANKED
								movwf		USB_desc_ptr, BANKED
								call		Descriptor		; get descriptor length
								movwf		USB_bytes_left, BANKED
								ifl USB_buffer_data+(wLength+1), ==, 0
								andiff USB_buffer_data+wLength, <, USB_bytes_left
									movf		USB_buffer_data+wLength, W, BANKED
									movwf		USB_bytes_left, BANKED
								endi
								call		SendDescriptorPacket
							endi
							break
						default
							bsf			USB_error_flags, 0, BANKED	; set Request Error flag
					ends
					break
				case GET_CONFIGURATION
					banksel		BD0IAH
					movf		BD0IAH, W, BANKED
					movwf		FSR0H, ACCESS
					movf		BD0IAL, W, BANKED
					movwf		FSR0L, ACCESS
					banksel		USB_curr_config
					movf		USB_curr_config, W, BANKED
					movwf		INDF0					; copy current device configuration to EP0 IN buffer
					banksel		BD0IBC
					movlw		0x01
					movwf		BD0IBC, BANKED			; set EP0 IN byte count to 1
					movlw		0xC8
					movwf		BD0IST, BANKED			; send packet as DATA1, set UOWN bit
					break
				case SET_CONFIGURATION
					ifl USB_buffer_data+wValue, <=, NUM_CONFIGURATIONS
						banksel		UEP1
						clrf		UEP1, BANKED		; clear all EP control registers except for EP0 to disable EP1-EP15 prior to setting configuration
						clrf		UEP2, BANKED
						clrf		UEP3, BANKED
						clrf		UEP4, BANKED
						clrf		UEP5, BANKED
						clrf		UEP6, BANKED
						clrf		UEP7, BANKED
						banksel		USB_buffer_data
						movf		USB_buffer_data+wValue, W, BANKED
						movwf		USB_curr_config, BANKED
						select
							case 0
								movlw		ADDRESS_STATE
								movwf		USB_USWSTAT, BANKED
;#ifdef SHOW_ENUM_STATUS
;								movlw		0xE0
;								andwf		PORTC, F, ACCESS
;								bsf			PORTC, 2, ACCESS
;#endif
								break
							default
								movlw		CONFIG_STATE
								movwf		USB_USWSTAT, BANKED
;#ifdef SHOW_ENUM_STATUS
;								movlw		0xE0
;								andwf		PORTC, F, ACCESS
;								bsf			PORTC, 3, ACCESS
;#endif
						ends
						banksel		BD0IBC
						clrf		BD0IBC, BANKED			; set byte count to 0
						movlw		0xC8
						movwf		BD0IST, BANKED			; send packet as DATA1, set UOWN bit
					otherwise
						bsf			USB_error_flags, 0, BANKED	; set Request Error flag
					endi
					break
				case GET_INTERFACE
					movf		USB_USWSTAT, W, BANKED
					select
						case CONFIG_STATE
							ifl USB_buffer_data+wIndex, <, NUM_INTERFACES
								banksel		BD0IAH
								movf		BD0IAH, W, BANKED
								movwf		FSR0H, ACCESS
								movf		BD0IAL, W, BANKED		; get buffer pointer
								movwf		FSR0L, ACCESS
								clrf		INDF0					; always send back 0 for bAlternateSetting
								movlw		0x01
								movwf		BD0IBC, BANKED			; set byte count to 1
								movlw		0xC8
								movwf		BD0IST, BANKED			; send packet as DATA1, set UOWN bit
							otherwise
								bsf			USB_error_flags, 0, BANKED	; set Request Error flag
							endi
							break
						default
							bsf			USB_error_flags, 0, BANKED	; set Request Error flag
					ends
					break
				case SET_INTERFACE
					movf		USB_USWSTAT, W, BANKED
					select
						case CONFIG_STATE
							ifl USB_buffer_data+wIndex, <, NUM_INTERFACES
								movf		USB_buffer_data+wValue, W, BANKED
								select
									case 0									; currently support only bAlternateSetting of 0
										banksel		BD0IBC
										clrf		BD0IBC, BANKED			; set byte count to 0
										movlw		0xC8
										movwf		BD0IST, BANKED			; send packet as DATA1, set UOWN bit
										break
									default
										bsf			USB_error_flags, 0, BANKED	; set Request Error flag
								ends
							otherwise
								bsf			USB_error_flags, 0, BANKED	; set Request Error flag
							endi
							break
						default
							bsf			USB_error_flags, 0, BANKED	; set Request Error flag
					ends
					break
				case SET_DESCRIPTOR
				case SYNCH_FRAME
				default
					bsf			USB_error_flags, 0, BANKED	; set Request Error flag
					break
			ends
			return

ClassRequests
			movf		USB_buffer_data+bRequest, W, BANKED
			select
                case SET_REPORT
                    ;btg         PORTA,6,ACCESS
                    banksel     USB_dev_req
                    movlw		HID_SET_REPORT
                    movwf		USB_dev_req, BANKED			; processing a SET_REPORT request
                    movf        USB_buffer_data+wLength,W,BANKED    ;max 256bytes in
                    movwf       USB_bytes_left,BANKED
                    banksel     PROG_add_lo
                    movlw       low PROG_add_lo
                    movwf       FSR2L
                    movlw       high PROG_add_lo
                    movwf       FSR2H
                    movlw       0xC8
                    banksel     BD0OST
                    movwf       BD0OST,BANKED
					break
				default
					bsf			USB_error_flags, 0, BANKED	; set Request Error flag
                    break
			ends
			return

VendorRequests
			movf		USB_buffer_data+bRequest, W, BANKED
			select
;				case SET_RA0
;					bsf			PORTB, 4, ACCESS		; set RA0 high
;					banksel		BD0IBC
;					clrf		BD0IBC, BANKED			; set byte count to 0
;					movlw		0xC8
;					movwf		BD0IST, BANKED			; send packet as DATA1, set UOWN bit
;					break
;				case CLR_RA0
;					bcf			PORTB, 4, ACCESS		; set RA0 low
;					banksel		BD0IBC
;					clrf		BD0IBC, BANKED			; set byte count to 0
;					movlw		0xC8
;					movwf		BD0IST, BANKED			; send packet as DATA1, set UOWN bit
;					break
				default
					bsf			USB_error_flags, 0, BANKED	; set Request Error flag
			ends
			return

ProcessInToken
			banksel		USB_USTAT
			movf		USB_USTAT, W, BANKED
			andlw		0x18		; extract the EP bits
			select
				case EP0
					movf		USB_dev_req, W, BANKED
					select
						case SET_ADDRESS
							movf		USB_address_pending, W, BANKED
							banksel		UADDR
							movwf		UADDR, BANKED
							banksel		USB_USTAT
							select
								case 0
									movlw		DEFAULT_STATE
									movwf		USB_USWSTAT, BANKED
;#ifdef SHOW_ENUM_STATUS
;									movlw		0xE0
;									andwf		PORTC, F, ACCESS
;									bsf			PORTC, 1, ACCESS
;#endif
									break
								default
									movlw		ADDRESS_STATE
									movwf		USB_USWSTAT, BANKED
;#ifdef SHOW_ENUM_STATUS
;									movlw		0xE0
;									andwf		PORTC, F, ACCESS
;									bsf			PORTC, 2, ACCESS
;#endif
							ends
							break
                        case HID_SET_REPORT
                            btg         PORTA,6,ACCESS          ; toggle RED to say programming
;                            movlw		NO_REQUEST
;                            movwf		USB_dev_req, BANKED		; clear device request
;                            banksel		BD0OBC
;                            movlw		MAX_PACKET_SIZE
;                            movwf		BD0OBC, BANKED
;                            movlw		0x88
;                            movwf		BD0OST, BANKED
;                            clrf		BD0IBC, BANKED		; set byte count to 0
;                            movlw		0xC8
;                            movwf		BD0IST, BANKED		; send packet as DATA1, set UOWN bit
;                            banksel     PROGRAM_IT
;                            incf        PROGRAM_IT,F,BANKED ; set program call.
                            break
						case GET_DESCRIPTOR
							call		SendDescriptorPacket
							break
					ends
					break
				case EP1
					break
				case EP2
					break
			ends
			return

ProcessOutToken
			banksel		USB_USTAT
			movf		USB_USTAT, W, BANKED
			andlw		0x18		; extract the EP bits
			select
				case EP0
                    movf        USB_dev_req, W, BANKED
                    select
                        case HID_SET_REPORT
                            banksel		USB_buffer_data
                            movf		USB_buffer_desc+ADDRESSH, W, BANKED
                            movwf		FSR0H, ACCESS
                            movf		USB_buffer_desc+ADDRESSL, W, BANKED
                            movwf		FSR0L, ACCESS

                            banksel		USB_bytes_left
                            ifl USB_bytes_left, <, MAX_PACKET_SIZE
                                movlw		NO_REQUEST
                                movwf		USB_dev_req, BANKED		; clear device request
                                banksel     PROGRAM_IT
                                incf        PROGRAM_IT,F,BANKED ; set program call.
                                movf		USB_bytes_left, W, BANKED
                            otherwise
                                movlw		MAX_PACKET_SIZE
                            endi
                            subwf		USB_bytes_left, F, BANKED
                            movwf		USB_packet_length, BANKED
                            banksel		USB_loop_index
                            forlf USB_loop_index, 1, USB_packet_length
                                movf        POSTINC0,W,ACCESS
                                movwf		POSTINC2			; copy to EP0 IN buffer, and increment FSR0
                            next USB_loop_index
                            banksel		BD0OST
                            movlw		0x40
                            xorwf		BD0OST, W, BANKED		; toggle the DATA01 bit
                            andlw		0x40					; clear the PIDs bits
                            iorlw		0x88					; set UOWN and DTS bits
                            movwf		BD0OST, BANKED
                            clrf		BD0IBC, BANKED		; set byte count to 0
                            movlw		0xC8
                            movwf		BD0IST, BANKED		; send packet as DATA1, set UOWN bit
                            break
                        default
                            banksel		BD0OBC
                            movlw		MAX_PACKET_SIZE
                            movwf		BD0OBC, BANKED
                            movlw		0x88
                            movwf		BD0OST, BANKED
                            clrf		BD0IBC, BANKED		; set byte count to 0
                            movlw		0xC8
                            movwf		BD0IST, BANKED		; send packet as DATA1, set UOWN bit
                            break
                    ends
					break
				case EP1
					break
				case EP2
					break
			ends
			return

; Come into here with USB_bytes_left being size and usb ptr set
SendDescriptorPacket
			banksel		USB_bytes_left
			ifl USB_bytes_left, <, MAX_PACKET_SIZE
				movlw		NO_REQUEST
				movwf		USB_dev_req, BANKED		; sending a short packet, so clear device request
				movf		USB_bytes_left, W, BANKED
			otherwise
				movlw		MAX_PACKET_SIZE
			endi
			subwf		USB_bytes_left, F, BANKED
			movwf		USB_packet_length, BANKED
			banksel		BD0IBC
			movwf		BD0IBC, BANKED			; set EP0 IN byte count with packet size
			movf		BD0IAH, W, BANKED		; put EP0 IN buffer pointer...
			movwf		FSR0H, ACCESS
			movf		BD0IAL, W, BANKED
			movwf		FSR0L, ACCESS			; ...into FSR0
			banksel		USB_loop_index
			forlf USB_loop_index, 1, USB_packet_length
				call		Descriptor			; get next byte of descriptor being sent
				movwf		POSTINC0			; copy to EP0 IN buffer, and increment FSR0
				incf		USB_desc_ptr, F, BANKED	; increment the descriptor pointer
			next USB_loop_index
			banksel		BD0IST
			movlw		0x40
			xorwf		BD0IST, W, BANKED		; toggle the DATA01 bit
			andlw		0x40					; clear the PIDs bits
			iorlw		0x88					; set UOWN and DTS bits
			movwf		BD0IST, BANKED
			return

APPLICATION	code
Erase_Block
            banksel     PROG_add_lo
            movf        PROG_add_lup,W,BANKED
            movwf       TBLPTRU
            movf        PROG_add_hi,W,BANKED
            movwf       TBLPTRH
            movf        PROG_add_lo,W,BANKED
            movwf       TBLPTRL                 ; load in address for erase
            bsf         EECON1, EEPGD           ; point to Flash program memory
            bcf         EECON1, CFGS            ; access Flash program memory
            bsf         EECON1, WREN            ; enable write to memory
            bsf         EECON1, FREE            ; enable block Erase operation
            ;bcf         INTCON, GIE             ; disable interrupts
            movlw       55h
            movwf       EECON2
            movlw       0AAh
            movwf       EECON2
            bsf         EECON1, WR              ; start erase (Cpu stall)
            return

Write_Block_64
;            bsf         PORTA,6,ACCESS          ; turn on RED, start programming
            banksel     PROG_add_lo
            movf        PROG_add_lup,W,BANKED
            movwf       TBLPTRU
            movf        PROG_add_hi,W,BANKED
            movwf       TBLPTRH
            movf        PROG_add_lo,W,BANKED
            movwf       TBLPTRL                 ; load in address
            movlw       low PROGRAM_data
            movwf       FSR0L
            movlw       high PROGRAM_data
            movwf       FSR0H
            banksel     COUNTER
            movlw       D'64'
            movwf       COUNTER
Write_Byte
            movf        POSTINC0,W
            movwf       TABLAT
            tblwt*+
            decfsz      COUNTER
            bra         Write_Byte
Prepare_Write_again
            banksel     PROG_add_lo
            movf        PROG_add_lup,W,BANKED
            movwf       TBLPTRU
            movf        PROG_add_hi,W,BANKED
            movwf       TBLPTRH
            movf        PROG_add_lo,W,BANKED
            movwf       TBLPTRL                 ; load in address
Program_Memory
            bsf         EECON1, EEPGD           ; point to Flash program memory
            bcf         EECON1, CFGS            ; access Flash program memory
            bsf         EECON1, WREN            ; enable write to memory
            movlw       55h
            movwf       EECON2
            movlw       0AAh
            movwf       EECON2
            bsf         EECON1, WR              ; start program (Cpu stall)
            bcf         EECON1, WREN            ; disable write to memory
;            bcf         PORTA,6,ACCESS          ; turn off RED, end programming
            return

Sanity_Check
            banksel     PROG_add_lo
            ifl PROG_add_lo,==,h'FF'
                ifl PROG_add_hi,==,h'FF'
                    ifl PROG_add_lup,==,h'FF'
                        ifl PROG_add_hup,==,h'FF'
                            banksel PROGRAM_IT
                            clrf        PROGRAM_IT, BANKED
                            bsf     WDTCON,SWDTEN,ACCESS
                            repeat
                            forever
                        endi
                    endi
                endi
            endi
            banksel     PROG_add_lo
            movf        PROG_add_lo,W,BANKED
            andlw       0x3F
            ifl     W,==,0
            otherwise
                movlw   0xC0
                andwf   PROG_add_lo,F,BANKED
            endi
            return



Main
;Initialize Port A
            ;bcf         WDTCON,SWDTEN,ACCESS
            clrf        INTCON,ACCESS
            clrf        INTCON2,ACCESS
            clrf        STKPTR,ACCESS
            movlw       b'01110000'
            movwf       OSCCON, ACCESS
            movlw       b'00010000'
            movwf       OSCCON2, ACCESS
osc_start:
            btfss       OSCCON2,PLLRDY,ACCESS
            goto        osc_start
            movlw       b'10010000'
            movwf       ACTCON,ACCESS

            banksel     LATA
            clrf        LATA
            clrf        LATB
            clrf        LATC

            movlw       0x80
			movwf		TRISB, ACCESS
            movwf       WPUB,ACCESS
            movlw       b'10100000'
            movwf       LATB,ACCESS
			clrf		TRISA, ACCESS			; set up all PORTA pins to be digital outputs
			clrf		TRISC, ACCESS
            clrf        ANSELA, ACCESS
            clrf        ANSELB, ACCESS
            clrf        ANSELC, ACCESS

            movlw       b'10001000'
            movwf       PMD0, ACCESS
            movlw       b'11111111'
            movwf       PMD1, ACCESS
			clrf		SLRCON, ACCESS
            clrf        ANSELA, ACCESS
            clrf        ANSELB, ACCESS
            clrf        ANSELC, ACCESS
            bsf			PORTA, 6, ACCESS        ; Red
            bsf			PORTC, 0, ACCESS        ; Green
            bsf			PORTC, 1, ACCESS        ; Blue
            ;repeat								; do nothing...
			;untilset    OSCCON,HFIOFS,ACCESS
            ;bcf			PORTA, 6, ACCESS
			clrf		ADCON0, ACCESS
			clrf		SLRCON, ACCESS
			clrf		CM1CON0, ACCESS
			clrf		CM2CON0, ACCESS

			;move to Application if the button is not pressed
            bsf         LATB,RB7,ACCESS
;loopb
;            btfss       PORTB,RB7,ACCESS
;            goto        loopb
            btfsc       PORTB,7,ACCESS
            goto        USER_APPLICATION

			call		InitUSB					; initialize the USB registers and serial interface engine
			repeat
				call		ServiceUSB			; service USB requests...
				banksel		USB_USWSTAT
			until USB_USWSTAT, ==, CONFIG_STATE	; ...until the host configures the peripheral
			bsf			PORTC, 1, ACCESS		; set Blue high
			banksel		COUNTER_L
			clrf		COUNTER_L, BANKED
			clrf		COUNTER_H, BANKED
			repeat
				banksel		COUNTER_L
				incf		COUNTER_L, F, BANKED
				ifset STATUS, Z, ACCESS
					incf		COUNTER_H, F, BANKED
				endi
                banksel     PROGRAM_IT
                ifl PROGRAM_IT,==,d'1'
                    bsf         PORTA,6,ACCESS          ; turn on RED, start programming
                    call        Sanity_Check
                    call        Erase_Block
                    call        Write_Block_64
                    clrf        PROGRAM_IT, BANKED
                    bcf         PORTA,6,ACCESS          ; turn off RED, end programming
                endi
                banksel     PROG_add_lo
                ifl PROG_add_lo,==,d'0'
                    bsf         PORTC,0,ACCESS
                otherwise
                    bcf         PORTC,0,ACCESS
                endi
;                ifl PROGRAM_data+d'63',==,d'15'
;                    bsf     PORTA,RA6,ACCESS
;                otherwise
;                    bcf     PORTA,RA6,ACCESS
;                endi
;				ifset COUNTER_H, 7, BANKED
;					bcf			PORTC, 1, ACCESS
;				otherwise
;					bsf			PORTC, 1, ACCESS
;				endi
				call		ServiceUSB
			forever

			end

stop:
            goto        stop