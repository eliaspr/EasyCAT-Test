#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "EasyCAT.h"


//********************************************************************************************
//                                                                                           *
// AB&T Tecnologie Informatiche - Ivrea Italy                                                *
// http://www.bausano.net                                                                    *
// https://www.ethercat.org/en/products/791FFAA126AD43859920EA64384AD4FD.htm                 *
//                                                                                           *  
//********************************************************************************************    
//                                                                                           *
// This software is distributed as an example, in the hope that it could be useful,          *
// WITHOUT ANY WARRANTY, even the implied warranty of FITNESS FOR A PARTICULAR PURPOSE       *
//                                                                                           *
//******************************************************************************************** 


//----- EasyCAT library for Raspberry PI V 2.1 200213 ----------------------------------------
//
// Changed SPI chip select to automatic 
// The SPI transfers are made through buffers       
// Function "SPIReadRegisterDirect" now read always 4 bytes, as for datasheet specs

//----- Derived from the AB&T EasyCAT Arduino shield library V 2.0 --------------------------------



//#define DEB                                                     // enable debug prints


//--------------------------------------------------------------------------------------------------

  inline static void SPI_BuffTransfer(char* Buff, uint32_t Len)   // static function for the SPI transfer	
  {                                                               //
    bcm2835_spi_transfern(Buff, Len);                             // transfer a in/out buffer through the SPI
  };                                                              //
  
//--------------------------------------------------------------------------------------------------


//---- constructors --------------------------------------------------------------------------------

EasyCAT::EasyCAT()                              //------- default constructor ---------------------- 
{                                               // 
  Sync_ = ASYNC;                                // if no synchronization mode is declared
                                                // ASYNC is the default
                                                //
  SCS_ = RPI_GPIO_P1_24;                        // if no chip select is declared                 
                                                // pin 24 (CE0) is the default
}                                               //


EasyCAT::EasyCAT(uint8_t SCS) 		              //------- SPI_CHIP_SELECT options -----------------
                                                //
{                                               //                              
  SCS_ = SCS;                                   //  initialize chip select  
                                                //
}                                               // 


EasyCAT::EasyCAT(SyncMode Sync)                 //-------Synchronization options ---------------------- 
                                                //   
                                                // we can choose between:
                                                // ASYNC   = free running i.e. no synchronization
                                                //           between master and slave (default)   
                                                //
                                                // DC_SYNC = interrupt is generated from the
                                                //           Distributed clock unit
                                                //
                                                // SM_SYNC = interrupt is generated from the
                                                //           Syncronizatiom manager 2 
{                                               //
  Sync_ = Sync;                                 //                                           
  SCS_ = RPI_GPIO_P1_24;                        // default chip select pin 24 (CE0)                 
                                                //                                            
}                                               //  


                                                //-- Synchronization and chip select options -----  
EasyCAT::EasyCAT(uint8_t SCS, SyncMode Sync) 	  //
                                                //				
{                                               //
  Sync_ = Sync;                                 //  
                                                //   
  SCS_ = SCS;                                   //  initialize chip select                                                 
                                                //
}                                               //  

  
//---- EasyCAT board initialization ---------------------------------------------------------------


bool EasyCAT::Init()
{
  #define Tout 1000
  
  ULONG TempLong;
  unsigned short i;
   
    // Use for testing
    //bcm2835_set_debug(1);

    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root ?\n");
      return false;
    }

    //Setup SPI pins
    if (!bcm2835_spi_begin())
    {
      printf("bcm2835_spi_begin failed. Are you running as root ?\n");
      return false;
    }

    // Set SPI bit order	
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST); 
    
    //  Set SPI data mode
    //	BCM2835_SPI_MODE0 = 0,  // CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                 
  
    //  Set SPI clock speed
    //	BCM2835_SPI_CLOCK_DIVIDER_16    = 16,      ///< 16 = 64ns = 15.625MHz
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);



                                                          //Enable management of CS pin
    if (SCS_ == RPI_GPIO_P1_24)
    {
      bcm2835_spi_chipSelect(BCM2835_SPI_CS0);            // enable CS0 and set polarity 
      bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW); 
      printf("SCS = CS0\n");            
    }
    else if (SCS_ == RPI_GPIO_P1_26)
    {
      bcm2835_spi_chipSelect(BCM2835_SPI_CS1);            // enable CS1 and set polarity
      bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, LOW);
      printf("SCS = CS1\n");                    
    }
    
    else                                                  // wrong CS pin 
    {                                                     // exit
      printf("only BCM2835_SPI_CS0 or BCM2835_SPI_CS1 allowed for SCS pin\n");
      return false;             
    }  
                                                          
  usleep(100000);					                                // delay of 100mS
 
  SPIWriteRegisterDirect (RESET_CTL, DIGITAL_RST);        // LAN9252 reset 
   
  i = 0;                                                  // reset timeout 
  do                                                      // wait for reset to complete
  {                                                       //
    i++;                                                  //
    TempLong.Long = SPIReadRegisterDirect (RESET_CTL);    //
  }while (((TempLong.Byte[0] & 0x01) != 0x00) && (i != Tout));    
                                                          //                                                       
  if (i == Tout)                                          // time out expired      
  {                                                       //   
    return false;                                         // initialization failed  
  }                                                         
  
  i = 0;                                                  // reset timeout  
  do                                                      // check the Byte Order Test Register
  {                                                       //
    i++;                                                  //      
    TempLong.Long = SPIReadRegisterDirect (BYTE_TEST);    //
  }while ((TempLong.Long != 0x87654321) && (i != Tout));  //    
                                                          //                                                            
  if (i == Tout)                                          // time out expired      
  {                                                       // 
    return false;                                         // initialization failed  
  }            
  
  i = 0;                                                  // reset timeout  
  do                                                      // check the Ready flag
  {                                                       //
    i++;                                                  //    
    TempLong.Long = SPIReadRegisterDirect (HW_CFG);       //
  }while (((TempLong.Byte[3] & READY) == 0) && (i != Tout));
                                                          //
  if (i == Tout)                                          // time out expired      
  {                                                       //
    return false;                                         // initialization failed  
  }            
  

  
#ifdef BYTE_NUM
  printf ("STANDARD MODE\n"); 
#else
  printf ("CUSTOM MODE\n"); 
#endif

  printf ("%u Byte Out\n",TOT_BYTE_NUM_OUT);  
  printf ("%u Byte In\n",TOT_BYTE_NUM_IN);      

  printf ("Sync = ");                                                              
                                                            
                                                          
  if ((Sync_ == DC_SYNC) || (Sync_ == SM_SYNC))           //--- if requested, enable --------   
  {                                                       //--- interrupt generation -------- 
  
    if (Sync_ == DC_SYNC)
    {                                                     // enable interrupt from SYNC 0
      SPIWriteRegisterIndirect (0x00000004, AL_EVENT_MASK, 4);  
                                                          // in AL event mask register, and disable 
                                                          // other interrupt sources    
      printf("DC_SYNC\n");                                                      
    }                                                       
                                                                                                         
    else
    {                                                     // enable interrupt from SM 0 event
      SPIWriteRegisterIndirect (0x00000100, AL_EVENT_MASK, 4);  
                                                          // in AL event mask register, and disable 
                                                          // other interrupt sources 
      printf("SM_SYNC\n");    
    }   
                                                         
    SPIWriteRegisterDirect (IRQ_CFG, 0x00000111);         // set LAN9252 interrupt pin driver  
                                                          // as push-pull active high
                                                          // (On the EasyCAT shield board the IRQ pin
                                                          // is inverted by a mosfet, so Arduino                                                        
                                                          // receives an active low signal)
                                                                        
    SPIWriteRegisterDirect (INT_EN, 0x00000001);          // enable LAN9252 interrupt      
  } 

  else
  {
    printf("ASYNC\n");
  }
  

  TempLong.Long = SPIReadRegisterDirect (ID_REV);         // read the chip identification 
  printf ("Detected chip ");                     		      // and revision, and print it
  printf ("%x ", TempLong.Word[1]);                       // out on the serial line
  printf (" Rev ");                                       //    
  printf ("%u \n", TempLong.Word[0]);                     //    
  
 
  #ifdef DEB                                              // debug     
    printf ("\nBytes in OUT");  
    
    printf ("%u \n", TOT_BYTE_NUM_OUT);                      
    #ifdef CUSTOM
       printf ("%u \n", TOT_BYTE_NUM_ROUND_OUT); 
    #else
       printf ("%u \n", TOT_BYTE_NUM_OUT);     
    #endif
    printf ("%u \n", FST_BYTE_NUM_OUT);                    
    printf ("%u \n", FST_BYTE_NUM_ROUND_OUT);                       
    printf ("%u \n", SEC_BYTE_NUM_OUT);                    
    printf ("%u \n", SEC_BYTE_NUM_ROUND_OUT);                       
              
    printf ("\nBytes in IN");  
    printf ("%u \n", TOT_BYTE_NUM_IN);    
    #ifdef CUSTOM
       printf ("%u \n", TOT_BYTE_NUM_ROUND_IN); 
    #else
       printf ("%u \n", TOT_BYTE_NUM_IN);     
    #endif
    printf ("%u \n", FST_BYTE_NUM_IN);                    
    printf ("%u \n", FST_BYTE_NUM_ROUND_IN);                       
    printf ("%u \n", SEC_BYTE_NUM_IN);                    
    printf ("%u \n", SEC_BYTE_NUM_ROUND_IN);                       
    printf ("\n");                                
  #endif
  
  return true;                                            // initalization completed        
};  


//---- EtherCAT task ------------------------------------------------------------------------------

unsigned char EasyCAT::MainTask()                           // must be called cyclically by the application

{
  bool WatchDog = true;
  bool Operational = false; 
  unsigned char i;
  ULONG TempLong; 
  unsigned char Status;  
 
  
  TempLong.Long = SPIReadRegisterIndirect (WDOG_STATUS, 1); // read watchdog status
  if ((TempLong.Byte[0] & 0x01) == 0x01)                    //
    WatchDog = false;                                       // set/reset the corrisponding flag
  else                                                      //
    WatchDog = true;                                        //
    
  TempLong.Long = SPIReadRegisterIndirect (AL_STATUS, 1);   // read the EtherCAT State Machine status
  Status = TempLong.Byte[0] & 0x0F;                         //
  if (Status == ESM_OP)                                     // to see if we are in operational state
    Operational = true;                                     //
  else                                                      // set/reset the corrisponding flag
    Operational = false;                                    //    

                                                            //--- process data transfer ----------
                                                            //                                                        
  if (WatchDog | !Operational)                              // if watchdog is active or we are 
  {                                                         // not in operational state, reset 
    for (i=0; i < TOT_BYTE_NUM_OUT ; i++)                   // the output buffer
    BufferOut.Byte[i] = 0;                                  //

 #ifdef DEB                                                 // only if debug is enabled
    if (!Operational)                                       //
      printf("Not operational\n");                          //
    if (WatchDog)                                           //    
      printf("WatchDog\n");                           	    //  
 #endif                                                     //
  }
  
  else                                                      
  {                                                         
    SPIReadProcRamFifo();                                   // otherwise transfer process data from 
  }                                                         // the EtherCAT core to the output buffer  
                 
  SPIWriteProcRamFifo();                                    // we always transfer process data from
                                                            // the input buffer to the EtherCAT core  

  if (WatchDog)                                             // return the status of the State Machine      
  {                                                         // and of the watchdog
    Status |= 0x80;                                         //
  }                                                         //
  return Status;                                            //   
}


//---- read a directly addressable registers  -----------------------------------------------------

unsigned long EasyCAT::SPIReadRegisterDirect (unsigned short Address)

                                                            // Address = register to read                                               
{
  ULONG Result; 
  UWORD Addr;
  Addr.Word = Address; 
  char LocalBuff[7];


  LocalBuff[0] = COMM_SPI_READ;                             // SPI read command
  LocalBuff[1] = Addr.Byte[1];                              // address of the register to read
  LocalBuff[2] = Addr.Byte[0];                              // MsByte first
  
  SPI_BuffTransfer(&LocalBuff[0], 7);                       // SPI transfer
 
  Result.Byte[0] = LocalBuff[3];                            // read the result
  Result.Byte[1] = LocalBuff[4];                            //
  Result.Byte[2] = LocalBuff[5];                            //
  Result.Byte[3] = LocalBuff[6];                            //
   
  return Result.Long;                                       // return the result  
}


//---- write a directly addressable registers  ----------------------------------------------------

void EasyCAT::SPIWriteRegisterDirect (unsigned short Address, unsigned long DataOut)

                                                            // Address = register to write
                                                            // DataOut = data to write
{ 
  ULONG Data; 
  UWORD Addr;
  Addr.Word = Address;
  Data.Long = DataOut;    
  char LocalBuff[7];   
  
  
  LocalBuff[0] = COMM_SPI_WRITE;                            // SPI write command
  LocalBuff[1] = Addr.Byte[1];                              // address of the register to write
  LocalBuff[2] = Addr.Byte[0];                              // MsByte first

  LocalBuff[3] = Data.Byte[0];                              // data to write 
  LocalBuff[4] = Data.Byte[1];                              // LsByte first
  LocalBuff[5] = Data.Byte[2];                              //
  LocalBuff[6] = Data.Byte[3];                              //
  
  SPI_BuffTransfer(&LocalBuff[0], 7);                       // SPI transfer
}


//---- read an indirectly addressable registers  --------------------------------------------------

unsigned long EasyCAT::SPIReadRegisterIndirect (unsigned short Address, unsigned char Len)

                                                            // Address = register to read
                                                            // Len = number of bytes to read (1,2,3,4)
                                                            //
                                                            // a long is returned but only the requested bytes
                                                            // are meaningful, starting from LsByte                                                  
{
  ULONG TempLong;
  UWORD Addr;
  Addr.Word = Address;
                                                            // compose the command
                                                            //
  TempLong.Byte[0] = Addr.Byte[0];                          // address of the register to read
  TempLong.Byte[1] = Addr.Byte[1];                          // LsByte first
  TempLong.Byte[2] = Len;                                   // number of bytes to read
  TempLong.Byte[3] = ESC_READ;                              // ESC read 

  SPIWriteRegisterDirect (ECAT_CSR_CMD, TempLong.Long);     // write the command

  do
  {                                                         // wait for command execution
    TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_CMD);    //
  }                                                         //
  while(TempLong.Byte[3] & ECAT_CSR_BUSY);                  //
                                                             
  TempLong.Long = SPIReadRegisterDirect(ECAT_CSR_DATA);     // read the requested register
  return TempLong.Long;                                     //
}


//---- write an indirectly addressable registers  -------------------------------------------------

void  EasyCAT::SPIWriteRegisterIndirect (unsigned long DataOut, unsigned short Address, unsigned char Len)

                                                            // Address = register to write
                                                            // DataOut = data to write                                                    
{
  ULONG TempLong;
  UWORD Addr;
  Addr.Word = Address;

  
  SPIWriteRegisterDirect (ECAT_CSR_DATA, DataOut);          // write the data

                                                            // compose the command
                                                            //                                
  TempLong.Byte[0] = Addr.Byte[0];                          // address of the register to write  
  TempLong.Byte[1] = Addr.Byte[1];                          // LsByte first
  TempLong.Byte[2] = Len;                                   // number of bytes to write
  TempLong.Byte[3] = ESC_WRITE;                             // ESC write

  SPIWriteRegisterDirect (ECAT_CSR_CMD, TempLong.Long);     // write the command

  do                                                        // wait for command execution
  {                                                         //
    TempLong.Long = SPIReadRegisterDirect (ECAT_CSR_CMD);   //  
  }                                                         //  
  while (TempLong.Byte[3] & ECAT_CSR_BUSY);                 //
}


//---- read from process ram fifo ----------------------------------------------------------------

void EasyCAT::SPIReadProcRamFifo()      // read data from the output process ram, through the fifo
                                        //        
                                        // these are the bytes received from the EtherCAT master and
                                        // that will be use by our application to write the outputs
{
  ULONG TempLong;
  char LocalBuff[64+3];  
  
  
  #if TOT_BYTE_NUM_OUT > 0

    SPIWriteRegisterDirect (ECAT_PRAM_RD_CMD, PRAM_ABORT);        // abort any possible pending transfer

  
    SPIWriteRegisterDirect (ECAT_PRAM_RD_ADDR_LEN, (0x00001000 | (((uint32_t)TOT_BYTE_NUM_OUT) << 16)));   
                                                                  // the high word is the num of bytes
                                                                  // to read 0xTOT_BYTE_NUM_OUT----
                                                                  // the low word is the output process        
                                                                  // ram offset 0x----1000 

    SPIWriteRegisterDirect (ECAT_PRAM_RD_CMD, 0x80000000);        // start command        
 
                                                //------- one round is enough if we have -----------
                                                //------- to transfer up to 64 bytes ---------------
   
    do                                                            // wait for the data to be       
    {                                                             // transferred from the output  
      TempLong.Long = SPIReadRegisterDirect (ECAT_PRAM_RD_CMD);   // process ram to the read fifo       
    }                                                             //    
    while (TempLong.Byte[1] != (FST_BYTE_NUM_ROUND_OUT/4));       //  
  
    LocalBuff[0] = COMM_SPI_READ;                                 // SPI read command
    LocalBuff[1] = 0x00;                                          // address of the read FIFO  
    LocalBuff[2] = 0x00;                                          // MsByte first
  
    SPI_BuffTransfer(&LocalBuff[0], 3+FST_BYTE_NUM_ROUND_OUT);    // SPI transfer

    memcpy((char*)&BufferOut.Byte[0], &LocalBuff[3], FST_BYTE_NUM_ROUND_OUT);
  #endif  

  
  #if SEC_BYTE_NUM_OUT > 0                      //-- if we have to transfer more then 64 bytes -----
                                                //-- we must do another round ----------------------
                                                //-- to transfer the remainig bytes ----------------

    do                                                            // wait for the data to be       
    {                                                             // transferred from the output  
      TempLong.Long = SPIReadRegisterDirect(ECAT_PRAM_RD_CMD);    // process ram to the read fifo 
    }                                                             //    
    while (TempLong.Byte[1] != SEC_BYTE_NUM_ROUND_OUT/4);         //

    LocalBuff[0] = COMM_SPI_READ;                                 // SPI read command
    LocalBuff[1] = 0x00;                                          // address of the read FIFO  
    LocalBuff[2] = 0x00;                                          // MsByte first
  
    SPI_BuffTransfer(&LocalBuff[0], 3+SEC_BYTE_NUM_ROUND_OUT);    // SPI transfer

    memcpy((char*)&BufferOut.Byte[64], &LocalBuff[3], SEC_BYTE_NUM_ROUND_OUT);    
  #endif  
}  


//---- write to the process ram fifo --------------------------------------------------------------

void EasyCAT::SPIWriteProcRamFifo()     // write data to the input process ram, through the fifo
                                        //    
                                        // these are the bytes that we have read from the inputs of our                   
                                        // application and that will be sent to the EtherCAT master
{
  ULONG TempLong;
  char LocalBuff[64+3];    
  
  
  #if TOT_BYTE_NUM_IN > 0  
  
    SPIWriteRegisterDirect (ECAT_PRAM_WR_CMD, PRAM_ABORT);        // abort any possible pending transfer
 
    SPIWriteRegisterDirect (ECAT_PRAM_WR_ADDR_LEN, (0x00001200 | (((uint32_t)TOT_BYTE_NUM_IN) << 16)));   
                                                                  // the high word is the num of bytes
                                                                  // to write 0xTOT_BYTE_NUM_IN----
                                                                  // the low word is the input process        
                                                                  // ram offset  0x----1200
                                                                                               
    SPIWriteRegisterDirect (ECAT_PRAM_WR_CMD, 0x80000000);        // start command  
  
  
                                                //------- one round is enough if we have -----------
                                                //------- to transfer up to 64 bytes ---------------
    
    do                                                            // check that the fifo has      
    {                                                             // enough free space 
      TempLong.Long = SPIReadRegisterDirect (ECAT_PRAM_WR_CMD);   //  
    }                                                             //  
    while (TempLong.Byte[1] < (FST_BYTE_NUM_ROUND_IN/4));         // 
    
    LocalBuff[0] = COMM_SPI_WRITE;                                // SPI write command
    LocalBuff[1] = 0x00;                                          // address of the write fifo  
    LocalBuff[2] = 0x20;                                          // MsByte first
    
    memcpy(&LocalBuff[3], (char*)&BufferIn.Byte[0], FST_BYTE_NUM_ROUND_IN);     
    
    SPI_BuffTransfer(&LocalBuff[0], 3+FST_BYTE_NUM_ROUND_IN);     // SPI transfer                  
  #endif        

  
  #if SEC_BYTE_NUM_IN > 0                       //-- if we have to transfer more then 64 bytes -----
                                                //-- we must do another round ----------------------
                                                //-- to transfer the remainig bytes ----------------

    do                                                            // check that the fifo has     
    {                                                             // enough free space       
      TempLong.Long = SPIReadRegisterDirect(ECAT_PRAM_WR_CMD);    // 
    }                                                             //  
    while (TempLong.Byte[1] < (SEC_BYTE_NUM_ROUND_IN/4));         // 
    
    LocalBuff[0] = COMM_SPI_WRITE;                                // SPI write command
    LocalBuff[1] = 0x00;                                          // address of the write fifo  
    LocalBuff[2] = 0x20;                                          // MsByte first
    
    memcpy(&LocalBuff[3], (char*)&BufferIn.Byte[64], SEC_BYTE_NUM_ROUND_IN);     
     
    SPI_BuffTransfer(&LocalBuff[0], 3+SEC_BYTE_NUM_ROUND_IN);     // SPI transfer     
  #endif     
}
