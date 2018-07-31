/* ========================================
*
* Copyright University of Auckland, 2016
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault
* Verified by   : N/A
* Creation date : 2018-07-19
* Version       : 0.1 (finished on ...)
* Modifications :
* Known bugs    :
*
*
* Possible Improvements
* PUT all char like separators, etc as #define
* Actually calculate and check the checksum
* replace all tacom to tachom
*
* Notes
*
*
* ========================================
*/

//Use the cydwr window to change the priority of an interrupt. Remember that 0 is the highest priority; and 7, the lowest
//priority. The Cortex-M0 supports interrupt nesting; see Nested Interrupts for details.


///////////////////////////////////////////////////////////////////Define/////////////////////////////////////////////////////////////
#define RxBufferSize	200

#define SOM_CHAR_SR	0x24 // '$' SR - Speed Request
#define SOM_CHAR_IM	0x26 // '&' IM – Integrity Monitor
#define SOM_CHAR_MC	0x23 // '#' MC – Manual Control (emergency procedure only, degraded state, IR is offline)

#define SOC_CHAR	0x2a // '*' Start of checksum
#define SEPARATOR_CHAR	0x2c // ',' Separate fields in messages

#define EOM_CHAR_SR	0x0A // '\n'
#define EMPTY_CHAR	0x00 // '\0'

/*
---------------------------------------------
| Verbose 1 | System Condition Good or Bad |
| Verbose 2 | Sensor Data                  |
| Verbose 3 | All Sensor Data              |
---------------------------------------------
*/
#define VERBOSE_LEVEL_X	0
#define VERBOSE_LEVEL_1	1
#define VERBOSE_LEVEL_2	2
#define VERBOSE_LEVEL_3	3

#define STATE_OK	        0
#define STATE_DANGEROUS	    1
#define STATE_CRITICAL	    2
#define STATE_REPEAT	    9 //Can be considered as "We don't know", good for initialization

///////////////////////////////////////////////////////////////////Include/////////////////////////////////////////////////////////////
// Our own libraries
#include "SensorThresholds.h"
#include "SimuSensorData.h"

//Existing libraries
#include <project.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>     /* atof */


///////////////////////////////////////////////////////////////////Structures////////////////////////////////////////////////////////////
typedef struct SensorData
{
	// indicates whether or not we have all the data from the sensor
	uint8 DataGatheringComplete;
	
	//Raw data from the sensors
    uint8   WI_bow;  //head
	uint8   WI_stern;  //bottom
	uint8   WI_starboard; //right	
	uint8   WI_port;   //left
    
	float   BatteryVoltage;
    float   BatteryAmperage;
	
    float   Temp_PortMotor;
    float   Temp_StarboardMotor;
    float   Temp_Ref;   
    
	float   Tacom_PortMotor;
    float   Tacom_StarboardMotor;
    
	float   WC_FlowRate;
	
	//-----------------
	
	// indicates whether or not we have compared all the sensors output to their range
	uint8 DataComparisionComplete;
	
	//SensorState: 0,1, or 2 (OK, dangerous or critical) for EACH sensor
	uint8   WI_bow_State;  //head
	uint8   WI_stern_State;  //bottom
	uint8   WI_starboard_State; //right	
	uint8   WI_port_State;   //left
    
	uint8   BatteryVoltage_State;
    uint8   BatteryAmperage_State;
    uint8   Temp_PortMotor_State;
    uint8   Temp_StarboardMotor_State;
    uint8   Temp_Ref_State;   
    uint8   Tacom_PortMotor_State;
    uint8   Tacom_StarboardMotor_State;
    uint8   WC_FlowRate_State;
	
	//TypeState: 0,1, or 2 (OK, dangerous or critical) for EACH TYPE of sensor, this is what verbose2 IM ask for
	uint8   WI_State;
	uint8	Tachom_State;	
    uint8   TempState;
	uint8   BallSwitch_Pitch_State;
	uint8   BallSwitch_Roll_State;
	//batt voltage, current, flowmeter are already their own type

    //-----------------
	
	// indicates whether or not we have checked the state of the USV
	uint8 CheckComplete;
    
    //-----------------
    
    uint8 USV_State;
	
}SensorData;

typedef struct RPiOrders_Struct
{
	int16     PWM_Right_Orders;
	int16     PWM_Left_Orders;
	
} RPiOrders_Struct;

RPiOrders_Struct RPiOrders;

// My factory defaults for the struct
// https://stackoverflow.com/questions/6891720/initialize-reset-struct-to-zero-null
static const struct RPiOrders_Struct EmptyRPiOrdersStruct = { 0,0 };
//myStructVariable = EmptyStruct;


///////////////////////////////////////////////////////////////////Functions Definitions/////////////////////////////////////////////////////////////
//INIT
void    InitEntireSystem (void);
void    GlobalInit_RPI(void);
void 	parse_SR(char message[RxBufferSize]);
void 	parse_IM(char message[RxBufferSize]);
void 	parse_MC(char message[RxBufferSize]);
//RPI BUFFER
uint8   IsCharReady(void);
char    GetRxChar(void);
//SENSORS
void readSensor(void);
//ACTIONS
void    Execute_SR(void);
void    Execute_IM(uint8 VerboseLevel, SensorData StructIn);
void    Execute_MC(void);

///////////////////////////////////////////////////////////////////Global Variables/////////////////////////////////////////////////////////////
//RPI Rx buffer
char    RPImessage[RxBufferSize];
char	ReceiveBuffer[RxBufferSize]; //Circular Buffer for RPI Rx messages
char	*RxReadIndex	= ReceiveBuffer;
char	*RxWriteIndex	= ReceiveBuffer;
int 	RPI_EOM_flag = 0; //End Of Message Flag : indicate to the main that a RPI message has been completely recieved
uint8   BuffCompletellyFilled = 0; //just an indication parameter

// My factory defaults for the struct
// https://stackoverflow.com/questions/6891720/initialize-reset-struct-to-zero-null
static const struct SensorData EmptyStruct = { STATE_REPEAT, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0,
    STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT,
    STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT, STATE_REPEAT};
//myStructVariable = EmptyStruct;

//Previous (and complete) sensor data
SensorData SD_old;

//New (and potentially incomplete) sensor data
SensorData SD_new;

///////////////////////////////////////////////////////////////////Interrupts handling/////////////////////////////////////////////////////////////
CY_ISR_PROTO(MyRxInt); //Here "proto" means we put the ISR code at the END of the main, with all the other functions


///////////////////////////////////////////////////////////////////Main/////////////////////////////////////////////////////////////
int main()
{  
    InitEntireSystem();
    char outputstring[40];
    sprintf(outputstring,"%%%f,*FF\r\n", 15.3);
    
	//===========================================================================================================================================================
	// Beginning of the loop
	
	for(;;)
	{

		//=========================================================================================
		/* RPI receiving*/
		//CYGlobalIntDisable;
		
        if(RPI_EOM_flag == 1) //We have detected an EOM while buffering the UART message
		{
			int cpt = 0;
			RPI_EOM_flag = 0;
			char temp;
			
            //Depile and dump a char from buff until empty or one of the SOM found
			do
			{
				temp = GetRxChar();
			}while((RxWriteIndex != RxReadIndex) & (temp != SOM_CHAR_SR) & (temp != SOM_CHAR_IM) & (temp != SOM_CHAR_MC));
			
			if ((temp == SOM_CHAR_SR) | (temp == SOM_CHAR_IM) | (temp == SOM_CHAR_MC)) // if any SOM found then we continue
			{
                switch(temp) // Check which type of message we have
                {
                    case SOM_CHAR_SR  : // '$' SR - Speed Request
                        RPImessage[0] = SOM_CHAR_SR;
				        cpt++;
                        
                        do //Depile and save a char from buff until \n found (we know it has arrived because of RPI_EOM_flag)
        				{
        					RPImessage[cpt] = GetRxChar();
        					cpt++;
        				}while((RPImessage[cpt-1] != EOM_CHAR_SR));
        				
    					parse_SR(RPImessage);
                        Execute_SR();

                        break; /* optional */
                        //END OF: SR - Speed Request
                	
                    case SOM_CHAR_IM  : // '&' IM – Integrity Monitor
                        
                        RPImessage[0] = SOM_CHAR_IM;
				        cpt++;
                        
                        do //Depile and save a char from buff until \n found (we know it has arrived because of RPI_EOM_flag)
        				{
        					RPImessage[cpt] = GetRxChar();
        					cpt++;
        				}while((RPImessage[cpt-1] != EOM_CHAR_SR));
        				
    					parse_IM(RPImessage);
//                        Execute_IM(VERBOSE_LEVEL_1, SD_old);
                        
                        break; /* optional */
                        //END OF: IM – Integrity Monitor
                    
                    
                    case SOM_CHAR_MC  : // '#' MC – Manual Control (emergency procedure only, degraded state, IR is offline)
                        
                        RPImessage[0] = SOM_CHAR_MC;
				        cpt++;
                        
                        do //Depile and save a char from buff until \n found (we know it has arrived because of RPI_EOM_flag)
        				{
        					RPImessage[cpt] = GetRxChar();
        					cpt++;
        				}while((RPImessage[cpt-1] != EOM_CHAR_SR));
        				
    					parse_MC(RPImessage);
                        Execute_MC();
                        
                        break; /* optional */
                        //END OF: MC – Manual Contro
                  
                   default :
                        // If you go there, there was a problem
                        CyDelayUs(1);                  
                }
			}
            memset(RPImessage, 0, RxBufferSize); // clean the message field anyway
			BuffCompletellyFilled = 0; // reinit the flag anyway
		}
		else
		{
			readSensor();
            CyDelay(100); // for simu only: slow down the process
		}
	}
}// END OF MAIN




///////////////////////////////////////////////////////////////////Functions/////////////////////////////////////////////////////////////
/* RPI UART */
uint8	IsCharReady(void)
{
	return !(RxWriteIndex == RxReadIndex);
}

char GetRxChar(void)
{
	char	Result;
	//while(!IsCharReady()) Wait();
	Result = *RxReadIndex;
	//*RxReadIndex =0;
	RxReadIndex++;
	if (RxReadIndex >= ReceiveBuffer + RxBufferSize) RxReadIndex = ReceiveBuffer;
	return Result;
}

CY_ISR(MyRxInt)
{

	char temp = {0};
	while(UART_RPI_ReadRxStatus() & UART_RPI_RX_STS_FIFO_NOTEMPTY)
	{
		temp  = UART_RPI_ReadRxData();

		if (temp != EMPTY_CHAR)
		{
			if (temp == EOM_CHAR_SR)
			{
				RPI_EOM_flag = 1;
			}
			
			*RxWriteIndex = temp;
			RxWriteIndex++;
			
			if (RxWriteIndex >= ReceiveBuffer + RxBufferSize)
			{
				RxWriteIndex = ReceiveBuffer;
				BuffCompletellyFilled = 1;
			}
		}
	}
}

void parse_SR(char message[RxBufferSize])  // SR - Speed Request
{
	if (strstr(message, ",")) // Find the "," character in the message haystack
	{
        //Check checksum
        const char *PATTERN1 = "*";
        const char *PATTERN2 = ( char * )EOM_CHAR_SR; //"\n";

        char *target = NULL;
        char *start, *end;

        start = strstr( message, PATTERN1 );
        if (start)
        {
            // PATTERN1 has been found, check that the 3 following char are: F, F, and \n
            start += 1;
            if (*start == 'F')
            {
                start += 1;
                if (*start == 'F')
                {
                    start += 1;
                    if (*start == '\n')
                    {
                        target = ( char * )"FF";
                    }
                }
            }
//            start += strlen( PATTERN1 );
//            end = strstr( message, PATTERN2 ); 
//            if (*end == EOM_CHAR_SR)
//            {
////                target = ( char * )malloc( end - start + 1 );
////                memcpy( target, start, end - start );
////                target[end - start] = '\0';
//                
//                for (i = 0; i < (end - start); i++)
//                {
//                   number[i] = start[i];
//                }
//            }
        }
        else // There was no start of checksum found, probably bad message
        {
            CyDelayUs(1); //just for debug
        }

        if (strncmp(target, "FF", 2)== 0 ) // We found the checksum
        {
            int16 temp1 = 0;
            int16 temp2 = 0;

    		char *p = message;

    		// get PWM1 order
    		p = strchr(p, '$')+1;
    		temp1 = atoi(p);
    		
    		// get PWM2 order
    		p = strchr(p, ',')+1;
    		temp2 = atoi(p);
            
            if (abs(temp1) < 255 && abs(temp2) < 255)
            {
                RPiOrders.PWM_Right_Orders = (uint8)temp1;
                RPiOrders.PWM_Left_Orders = (uint8)temp2;
            }
            else // if the orders are outside range, stop the motors
            {
                RPiOrders.PWM_Right_Orders = 0u;
                RPiOrders.PWM_Left_Orders = 0u;
            }
        }
        else // if the orders are outside range, stop the motors
        {
            RPiOrders.PWM_Right_Orders = 0u;
            RPiOrders.PWM_Left_Orders = 0u;
        }
        free( target );
	}
}

void parse_IM(char message[RxBufferSize]) // IM - Integrity monitor
{
    //The 1st thing is to find the desired verbose level
    
    uint8 DesiredVerboseLevel = VERBOSE_LEVEL_X;

    //Check checksum
    const char *PATTERN1 = "*";
    const char *PATTERN2 = ( char * )EOM_CHAR_SR; //"\n";

    char *target = NULL;
    char *start, *end;

    start = strstr( message, PATTERN1 );
    if (start)
    {
        // PATTERN1 has been found, check that the 3 following char are: F, F, and \n
        start += 1;
        if (*start == 'F')
        {
            start += 1;
            if (*start == 'F')
            {
                start += 1;
                if (*start == '\n')
                {
                    target = ( char * )"FF";
                }
            }
        }
//            start += strlen( PATTERN1 );
//            end = strstr( message, PATTERN2 ); 
//            if (*end == EOM_CHAR_SR)
//            {
////                target = ( char * )malloc( end - start + 1 );
////                memcpy( target, start, end - start );
////                target[end - start] = '\0';
//                
//                for (i = 0; i < (end - start); i++)
//                {
//                   number[i] = start[i];
//                }
//            }
    }
    else // There was no start of checksum found, probably bad message
    {
        CyDelayUs(1); //just for debug
    }


    if (strncmp(target, "FF", 2) == 0) // We found the checksum
    {
        int16 temp1 = 0;
        int16 temp2 = 0;

		char *p = message;

		// get desired verbose level
		p = strchr(p, SOM_CHAR_IM) + 1;
		DesiredVerboseLevel = atoi(p);
        
        if ((DesiredVerboseLevel != VERBOSE_LEVEL_X) & (DesiredVerboseLevel != VERBOSE_LEVEL_1) &
        (DesiredVerboseLevel != VERBOSE_LEVEL_2) & (DesiredVerboseLevel != VERBOSE_LEVEL_3))
        {
            DesiredVerboseLevel = VERBOSE_LEVEL_X;
        }
    }
    
    free( target );
    
    SD_old.CheckComplete = STATE_OK; // for simu only
    Execute_IM(DesiredVerboseLevel, SD_old);
    SD_old.CheckComplete = STATE_REPEAT; // for simu only
}

void parse_MC(char message[RxBufferSize])
{
    if (strstr(message, ",")) // Find the "," character in the message haystack
	{
        //Check checksum
        const char *PATTERN1 = "*";
        const char *PATTERN2 = ( char * )EOM_CHAR_SR; //"\n";

        char *target = NULL;
        char *start, *end;

        start = strstr( message, PATTERN1 );
        if (start)
        {
            // PATTERN1 has been found, check that the 3 following char are: F, F, and \n
            start += 1;
            if (*start == 'F')
            {
                start += 1;
                if (*start == 'F')
                {
                    start += 1;
                    if (*start == '\n')
                    {
                        target = ( char * )"FF";
                    }
                }
            }
//            start += strlen( PATTERN1 );
//            end = strstr( message, PATTERN2 ); 
//            if (*end == EOM_CHAR_SR)
//            {
////                target = ( char * )malloc( end - start + 1 );
////                memcpy( target, start, end - start );
////                target[end - start] = '\0';
//                
//                for (i = 0; i < (end - start); i++)
//                {
//                   number[i] = start[i];
//                }
//            }
        }
        else // There was no start of checksum found, probably bad message
        {
            CyDelayUs(1); //just for debug
        }

        if (strncmp(target, "FF", 2)== 0 ) // We found the checksum
        {
            CyDelay(100);
        }
        free( target );
	}
}

void InitEntireSystem (void)
{
	//RPI
    //----------------------
    memset(RPImessage, 0, RxBufferSize); // clean the message field
    memset(ReceiveBuffer, 0, RxBufferSize); // clean the message field
    RPI_EOM_flag = 0;
    UART_RPI_Start();
    Rx_Int_StartEx(MyRxInt);
    
    SD_old = EmptyStruct;
	SD_new = EmptyStruct;
    
    RPiOrders = EmptyRPiOrdersStruct;

    CYGlobalIntEnable; //Enable the Global interrupts
    
}

void    Execute_SR(void)  // Modify motor speed, //OPTIMISE w/Global
{
    char outputstring[40];
    sprintf(outputstring,"Speed request recieved: %i,%i \r\n", RPiOrders.PWM_Right_Orders, RPiOrders.PWM_Left_Orders); 
    UART_RPI_PutString(outputstring);
    CyDelay(100);
}

void    Execute_IM(uint8 VerboseLevel, SensorData StructIn) // Answer the RPi, //OPTIMISE w/Global
{
    char outputstring[90];
    memset(outputstring, 0, sizeof(outputstring)); // clean the message field
    
    
    //Before Answering we check that the data is ready
    if (StructIn.CheckComplete == STATE_OK)
    {
    
    /*Check verbose level
        ---------------------------------------------
        | Verbose 1 | System Condition Good or Bad |
        | Verbose 2 | Sensor Data                  |
        | Verbose 3 | All Sensor Data              |
        ---------------------------------------------
    */
        switch(VerboseLevel) 
        {
            case VERBOSE_LEVEL_X  :
                sprintf(outputstring,"%%%i*FF\r\n", VerboseLevel); //%% means print '%'
                UART_RPI_PutString(outputstring);
                break; /* optional */
            
            case VERBOSE_LEVEL_1  : 
                /*
                Just the state of the USV
                */
                sprintf(outputstring,"%%%i,%i*FF\r\n", VerboseLevel, StructIn.USV_State); //%% means print '%'
                UART_RPI_PutString(outputstring);
                break; /* optional */
        	
            case VERBOSE_LEVEL_2  :
                /*
                Just the state of the TYPE of sensors
                uint8   WI_State;
            	uint8	Tachom_State;	
                uint8   TempState;
                uint8   BatteryVoltage_State;
                uint8   BatteryAmperage_State;
                uint8   WC_FlowRate_State;
                */

                sprintf(outputstring,"%%%i,%i,%i,%i,%i,%i,%i*FF\r\n", VerboseLevel, StructIn.WI_State, StructIn.Tachom_State, StructIn.TempState, 
                    StructIn.BatteryVoltage_State, StructIn.BatteryAmperage_State, StructIn.WC_FlowRate_State); //%% means print '%'
                UART_RPI_PutString(outputstring);
                break; /* optional */
            
            case VERBOSE_LEVEL_3  :
                /*
                
                Water ingress detected bow (boolean), 0 is NO water (ok) 1 is water detected (not ok)
                Water ingress detected stern (boolean), 0 is NO water (ok) 1 is water detected (not ok)
                Water ingress detected starboard (boolean), 0 is NO water (ok) 1 is water detected (not ok)
                Water ingress detected port (boolean), 0 is NO water (ok) 1 is water detected (not ok)
                
                Temperature of the center
                Temperature of the right motor
                Temperature of the left motor
                
                Battery level
                Instant current
                
                Speed of right motor as count of the IR sensor
                Speed of left motor as count of the IR sensor
                
                flow rate speed
                */
                sprintf(outputstring,"%%%i,%i,%i,%i,%i,%5.2f,%5.2f,%5.2f,%5.1f,%5.2f,%5.1f,%5.1f*FF\r\n", VerboseLevel, StructIn.WI_bow, StructIn.WI_stern, StructIn.WI_starboard,
                    StructIn.WI_port, StructIn.Temp_Ref, StructIn.Temp_StarboardMotor, StructIn.Temp_PortMotor, StructIn.BatteryVoltage, StructIn.BatteryAmperage,
                    StructIn.Tacom_StarboardMotor, StructIn.Tacom_PortMotor); //%% means print '%'
                UART_RPI_PutString(outputstring);
                //break; /* optional */
          
            default : 
                // If we can't recognise the VerboseLevel
                CyDelayUs(1); //Do nothing
        }
    }
    else //Ask the RPI to repeat because not ready
    {
        sprintf(outputstring,"%%%i,%i*FF\r\n", VerboseLevel, STATE_REPEAT); //%% means print '%'
        UART_RPI_PutString(outputstring);
    }
}

void    Execute_MC(void) // Modify motor speed
{
    CyDelay(100);
}


void readSensor(void) //OPTIMISE w/Global
{
    //Put new in old IF it was complete
    if (SD_new.CheckComplete)
    {
        SD_old = SD_new; //Replace all the fields of the OLD struct with the NEW one 
        SD_new = EmptyStruct; //Clean the new struct
    }
    
    // Read input : Raw data from the sensors, for simu only
    SD_new.WI_bow					= WI_BOW_CURRENTDATA	        ;  //head
    SD_new.WI_stern				    = WI_STERN_CURRENTDATA	    ;  //bottom
    SD_new.WI_starboard			    = WI_PORT_CURRENTDATA	        ; //right	
    SD_new.WI_port					= WI_STARBOARD_CURRENTDATA	;   //left
    SD_new.BatteryVoltage			= BATTERY_VOLATGE_CURRENTDATA	;
    SD_new.BatteryAmperage			= BATTERY_AMPERAGE_CURRENTDATA;
    SD_new.Temp_PortMotor			= THEMAL_PORT_CURRENTDATA	     ;
    SD_new.Temp_StarboardMotor		= THEMAL_STARTBOARD_CURRENTDATA;
    SD_new.Temp_Ref				    = THEMAL_CENTER_CURRENTDATA	 ;   
    SD_new.Tacom_PortMotor			= WC_FLOWRATE_CURRENTDATA		;
    SD_new.Tacom_StarboardMotor	    = TACOM_STARTBOARD_CURRENTDATA;	
    SD_new.WC_FlowRate				= TACOM_PORT_CURRENTDATA	     ;
    
    SD_new.DataGatheringComplete = 1;
    
	//Combine per sensor type
	SD_new.DataComparisionComplete = 1;
    
    //Calculate the state of the USV
    SD_new.CheckComplete = 1;
    
}

/* [] END OF FILE */
