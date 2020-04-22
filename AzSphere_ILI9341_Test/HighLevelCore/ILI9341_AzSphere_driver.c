
// Global Includes
#include <time.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <applibs/log.h>
#include "applibs_versions.h"
#include "ILI9341_AzSphere_driver.h"
#include "hw/avnet_mt3620_sk.h"

/// <summary>
/// Termination codes for this application. These are used for the
/// application exit code.  They they must all be between zero and 255,
/// where zero is reserved for successful termination.
/// </summary>
typedef enum {
    ExitCode_Success = 0,

    ExitCode_TermHandler_SigTerm = 1,

    ExitCode_Gpio_Open = 1,

    ExitCode_AccelTimerHandler_Consume = 2,
    ExitCode_AccelTimerHandler_ReadStatus = 3,
    ExitCode_AccelTimerHandler_ReadZAcceleration = 4,

    ExitCode_ReadWhoAmI_WriteThenRead = 5,
    ExitCode_ReadWhoAmI_WriteThenReadWrongWhoAmI = 6,
    ExitCode_ReadWhoAmI_InitTransfers = 7,
    ExitCode_ReadWhoAmI_TransferSequential = 8,
    ExitCode_ReadWhoAmI_TransferSequentialWrongWhoAmI = 9,

    ExitCode_Reset_InitTransfers = 10,
    ExitCode_Reset_TransferSequentialReset = 11,
    ExitCode_Reset_TransferSequentialSetRange = 12,

    ExitCode_Init_EventLoop = 13,
    ExitCode_Init_AccelTimer = 14,
    ExitCode_Init_InitConfig = 15,
    ExitCode_Init_OpenSpiMaster = 16,
    ExitCode_Init_SetBusSpeed = 17,
    ExitCode_Init_SetMode = 18,

    ExitCode_Main_EventLoopFail = 19
} ExitCode;

//Constants Defined for Custom Delay Generation
const struct timespec delay_1s = { 1, 0 };
const struct timespec delay_1us = { 0, 1000 };
const struct timespec delay_1ms = { 0, 1000000 };

//Constructs for easy setting up of ILI9341 Driver registers
uint8_t Power_Control_A[5] = { 0x39, ILI9341_RAMWR, ILI9341_NOP, 0x34, 0x02 }; // Power Controller A : CMD CBh
uint8_t Power_Control_B[3] = { ILI9341_NOP , ILI9341_PWCTR2 , ILI9341_PTLAR }; // Power Controller B : CMD CFh
uint8_t Driver_Timing_Control_A[3] = {0x85 , ILI9341_NOP , 0x78}; // DRIVER TIMING CONTROL A : Cmd E8h
uint8_t Driver_Timing_Control_B[2] = { ILI9341_NOP , ILI9341_NOP }; // DRIVER TIMING CONTROL B : Cmd EAh
uint8_t Power_On_Sequence_Control[4] = {0x64 , 0x03 , ILI9341_PTLON , 0x81}; //  : Cmd EDh
uint8_t Pump_Ratio_Control = ILI9341_INVOFF; //  : Cmd F7h
uint8_t Power_Control_VRH5_0 = 0x23; //  : Cmd C0h
uint8_t Power_Control_SAP2_0_BT3_0 = ILI9341_SLPIN; //  : Cmd C1h
uint8_t VCM_Control[2] = {0x3E , ILI9341_DISPOFF }; //  : Cmd C5h
uint8_t VCM_Control_2 = 0x86; //  : Cmd C7h
uint8_t Memory_Access_Control = 0x48; //  : Cmd 36h
uint8_t Pixel_Format = 0x55; //  : Cmd 3Ah
uint8_t Frame_Ratio_Control[2] = { ILI9341_NOP , 0x18}; // Standard RGB Format : Cmd B1h
uint8_t Display_Function_Control[3] = {0x08 , 0x82 , 0x27}; //  : Cmd B6h
uint8_t Gamma_Function_Disable = ILI9341_NOP; //  : Cmd F2h
uint8_t Gamma_Curve_Select = ILI9341_SWRESET; //  : Cmd 26h
uint8_t Positive_Gamma_Correction[15] = { ILI9341_RDSELFDIAG , 0x31, ILI9341_PASET , ILI9341_RDPIXFMT , 0x0E , 0x08 , 0x4E , 0xF1 , ILI9341_VSCRSADD , 0x07, ILI9341_SLPIN , 0x03 , 0x0E , ILI9341_RDDST , ILI9341_NOP }; //  : Cmd E0h
uint8_t Negative_Gamma_Correction[15] = { ILI9341_NOP , 0x0E , 0x14 , 0x03 , ILI9341_SLPOUT , 0x07 , 0x31 , ILI9341_PWCTR2 , 0x48 , 0x08 , ILI9341_RDSELFDIAG , ILI9341_RDPIXFMT , 0x31 , ILI9341_MADCTL , ILI9341_RDSELFDIAG }; //  : Cmd E1h
uint8_t Exit_Sleep = 0x00; //  : Cmd 11h
uint8_t Turn_On_Display = 0x00; //  : Cmd 29h


//Global Variables
SPIMaster_Config config;
uint8_t *spiReadData;

//Setting fp values 
rstfp = -1;
dcfp = -1;
ledfp = -1;
max_x = 0;
max_y = 0;

/**
  * @brief  Custom MilliSec Delay generator
  * @param  Int value coorosponding to delay generated in ms.
  * @retval None.
  */
extern void msleep(unsigned int tms) {
    while (tms != 0) {
        nanosleep(&delay_1ms, NULL);
        --tms;
    }
}
/*
extern int ILI9341_Set_Address(int fd, uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2) {
    int ret;
    // Sending Coloumn Addresses
    ret = spi_write(fd, ILI9341_COMMAND, Column_Address_Set_cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Column_Address_Set_cmd : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, X1 >> 8, 1);
    ret += spi_write(fd, ILI9341_DATA, X1, 1);
    ret += spi_write(fd, ILI9341_DATA, X2 >> 8, 1);
    ret += spi_write(fd, ILI9341_DATA, X2, 1);
    Log_Debug("INFO : ili9341_param_init : Column_Addreses : Bytes Tfr %d\n", ret);

    // Sending Page_Address_Set_Cmd
    ret = spi_write(fd, ILI9341_COMMAND, Page_Address_Set_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Page_Address_Set_Cmd : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, Y1 >> 8, 1);
    ret += spi_write(fd, ILI9341_DATA, Y1, 1);
    ret += spi_write(fd, ILI9341_DATA, Y2 >> 8, 1);
    ret += spi_write(fd, ILI9341_DATA, Y2, 1);
    Log_Debug("INFO : ili9341_param_init : Page_Address : Bytes Tfr %d\n", ret);

    // Sending Memory_Write_Cmd
    ret = spi_write(fd, ILI9341_COMMAND, Memory_Write_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Memory_Write_Cmd : Bytes Tfr %d\n", ret);
    
    if (ret < 0) {
        Log_Debug("ERROR : ILI9341_Set_Address : SETTING Addresses for Image");
        return ExitCode_Reset_TransferSequentialReset;
    }
    return ExitCode_Success;
}

///Sends block colour information to LCD
extern int ILI9341_Draw_Colour_Burst(int fd, uint16_t Colour, uint32_t Size)
{
    //SENDS COLOUR
    int ret;
    uint32_t Buffer_Size = 0;
    if ((Size * 2) < 500)
    {
        Buffer_Size = Size;
    }
    else
    {
        Buffer_Size = 500;
    }

//    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);

    unsigned char chifted = Colour >> 8;;
    unsigned char burst_buffer[Buffer_Size];
    for (uint32_t j = 0; j < Buffer_Size; j += 2)
    {
        burst_buffer[j] = chifted;
        burst_buffer[j + 1] = Colour;
    }

    uint32_t Sending_Size = Size * 2;
    uint32_t Sending_in_Block = Sending_Size / Buffer_Size;
    uint32_t Remainder_from_block = Sending_Size % Buffer_Size;
    ret = 0;
    if (Sending_in_Block != 0)
    {
        for (uint32_t j = 0; j < (Sending_in_Block); j++)
        {
            ret += spi_write(fd, ILI9341_DATA, &burst_buffer, sizeof(burst_buffer));
        }
    }

    //REMAINDER!
    ret += spi_write(fd, ILI9341_DATA, &burst_buffer, sizeof(Remainder_from_block));
    //HAL_SPI_Transmit(HSPI_INSTANCE, (unsigned char*)burst_buffer, Remainder_from_block, 10);

   // HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
    return ExitCode_Success;
}

extern int ILI9341_Fill_Screen(int fp, uint16_t Colour) {
    ILI9341_Set_Address(fp, 0, 0, ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT);
    ILI9341_Draw_Colour_Burst(fp, Colour, ILI9341_TFTWIDTH * ILI9341_TFTHEIGHT);
    return ExitCode_Success;
}
*/
//DRAW PIXEL AT XY POSITION WITH SELECTED COLOUR
//
//Location is dependant on screen orientation. x0 and y0 locations change with orientations.
//Using pixels to draw big simple structures is not recommended as it is really slow
//Try using either rectangles or lines if possible
//
void ILI9341_Draw_Pixel(int fd, uint16_t X, uint16_t Y, uint16_t Colour)
{
    if ((X >= ILI9341_TFTWIDTH) || (Y >= ILI9341_TFTHEIGHT)) return -1;	//OUT OF BOUNDS!

    int ret;
    uint16_t xA[4] = { (X >> 8) , X, ((X + 1) >> 8) , (X + 1)};
    uint16_t yA[4] = { (Y >> 8) , Y, ((Y + 1) >> 8) , (Y + 1) };

    ili9341_reset(rstfp);
    // Sending Coloumn Addresses
    ret = spi_write(fd, ILI9341_COMMAND, Column_Address_Set_cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Column_Address_Set_cmd : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, xA[0], 1);
    ret += spi_write(fd, ILI9341_DATA, xA[1], 1);
    ret += spi_write(fd, ILI9341_DATA, xA[2], 1);
    ret += spi_write(fd, ILI9341_DATA, xA[3], 1);
    Log_Debug("INFO : ili9341_param_init : Column_Addreses : Bytes Tfr %d\n", ret);

    ili9341_reset(rstfp);
    // Sending Page_Address_Set_Cmd
    ret = spi_write(fd, ILI9341_COMMAND, Page_Address_Set_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Page_Address_Set_Cmd : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, &yA[0], 1);
    ret += spi_write(fd, ILI9341_DATA, &yA[1], 1);
    ret += spi_write(fd, ILI9341_DATA, &yA[2], 1);
    ret += spi_write(fd, ILI9341_DATA, &yA[3], 1);
    Log_Debug("INFO : ili9341_param_init : Page_Address : Bytes Tfr %d\n", ret);

    ili9341_reset(rstfp);
    // Sending Memory_Write_Cmd
    ret = spi_write(fd, ILI9341_COMMAND, Memory_Write_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Memory_Write_Cmd : Bytes Tfr %d\n", ret);

    if (ret < 0) {
        Log_Debug("ERROR : ILI9341_Set_Address : SETTING Addresses for Image");
        return ExitCode_Reset_TransferSequentialReset;
    }

    //COLOUR	
    ili9341_reset(rstfp);
    uint8_t Temp_Buffer2[2] = { (Colour >> 8), Colour };
    ret = spi_write(fd, ILI9341_DATA, &Temp_Buffer2[0], 1);
    ret = spi_write(fd, ILI9341_DATA, &Temp_Buffer2[1], 1);
  // HAL_SPI_Transmit(HSPI_INSTANCE, Temp_Buffer2, 2, 1);
   // HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);

}

/**
  * @brief  Reset ili9341 I/F and display Module.
  * @param  None.
  * @retval None.
  */
extern uint8_t ili9341_reset(int rfd) {
    int ret = GPIO_SetValue(rfd, GPIO_Value_Low);
    if (ret != 0) {
        Log_Debug("ERROR: ili9341_reset : Could not set rstfp gpio\n");
        return ExitCode_Gpio_Open;
    }

    msleep(10);

    ret = GPIO_SetValue(rfd, GPIO_Value_High);
    if (ret != 0) {
        Log_Debug("ERROR: ili9341_reset : Could not set rstfp gpio\n");
        return ExitCode_Gpio_Open;
    }// return close(spiFd);
    return ExitCode_Success;
}

/**
  * @brief  sest Orientation of ili9341 display Module.
  * @param  None.
  * @retval None.
  */
extern uint8_t ili9341_orientation(int orient) {
    if (orient == ILI9341_ORIENT_POTRAIT) {
        max_x = ILI9341_TFTWIDTH;
        max_y = ILI9341_TFTHEIGHT;
    }
    else if (orient == ILI9341_ORIENT_LANDSCAPE) {
        max_x = ILI9341_TFTHEIGHT;
        max_y = ILI9341_TFTWIDTH;
    }
    else Log_Debug("WARNING : ILLIGAL ORIENTATION TYPE USING DEFAULT ORIENTATION LANDSCAPE");
    max_x = ILI9341_TFTHEIGHT;
    max_y = ILI9341_TFTWIDTH;
}

/**
  * @brief  Spi Write Command.
  * @param  None.
  * @retval None.
  */

int spi_write(int fd, uint16_t cmd, uint8_t b, int size)
{
    const size_t       transferCount = 1;
    SPIMaster_Transfer transfers;
    ssize_t            transferredBytes;
    
    int result = SPIMaster_InitTransfers(&transfers, transferCount);
    if (result != 0) {
        return -1;
    }

    transfers.flags = SPI_TransferFlags_Write;
    transfers.writeData = &b;
    transfers.length = sizeof(b);

    if (cmd == 1) //if sending a Command
        GPIO_SetValue(dcfp, GPIO_Value_Low);

    transferredBytes = SPIMaster_TransferSequential(fd, &transfers, transferCount);

    if (cmd == 1) // changing to Data 
        GPIO_SetValue(dcfp, GPIO_Value_High);
    
    transferredBytes == transfers.length;
    return transferredBytes;
}

/*
//  SPI Write Different impl
uint8_t spi_write(int fd,  uint8_t reg, uint8_t *b, uint16_t siz)
{
    SPIMaster_Transfer transfer;
    ssize_t            bytesTxed;
    uint8_t*           tbuff;
    size_t             tsize = (size_t) siz + 1;

    tbuff = (uint8_t*)malloc(tsize);
    tbuff[0] = reg & 0x7f;
    memcpy(&tbuff[1], b, siz);

    if( SPIMaster_InitTransfers(&transfer,1) != 0 )
        return (uint8_t)-1;

    transfer.flags = SPI_TransferFlags_Write;
    transfer.writeData = tbuff;
    transfer.length = tsize;

    bytesTxed = SPIMaster_TransferSequential(fd, &transfer, 1);
    if (bytesTxed < 0) {
        Log_Debug("ERROR: SPIMaster_TransferSequential:\n");
        return (uint8_t)-1;
        }

    free(tbuff);
    return(bytesTxed != tsize);
}
*/

/**
  * @brief  Spi Read Command.
  * @param  None.
  * @retval None.
  */
int spi_read(int fb, uint8_t b, uint8_t reg, uint16_t siz)
{
    uint8_t i;

    //reg |= 0x80;
    if ((i = (uint8_t)SPIMaster_WriteThenRead(fb, &reg, 1, &b, siz)) < 0) {
        Log_Debug("ERROR: SPI Read : unable to WriteThenRead\n");
        return ExitCode_ReadWhoAmI_WriteThenRead;
    }

    return ExitCode_Success;
}

static ExitCode ReadRDDIDIF_ID2(int fd)
{
    // ILI9341_DS_V1.02, 8.2.3., Read display identification information (DBh); 
    // Returns The 2nd parameter(ID1[7:0]) : LCD module’s manufacturer ID.
    int ret;
    static const uint8_t RDDIDIFReadCmd = ILI9341_RDID2;
    spiReadData = (uint8_t*)malloc(sizeof(uint8_t));
    ret = spi_read(fd, spiReadData, RDDIDIFReadCmd, sizeof(spiReadData));
    if (ret == 0) {
        Log_Debug("INFO : SPI READ of RDDID DBh is %x\n", spiReadData);
    }
    else {
        Log_Debug("ERROR : SPI READ of RDDID DBh Failed\n");
    }
    return ExitCode_Success;
}

/// <summary>
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static ExitCode ReadRDDIDIF(int fb)
{
    // ILI9341_DS_V1.02, 8.2.3., Read display identification information (04h); 
    // Returns four parameters :    The 1st parameter is dummy data.
    //                              The 2nd parameter(ID1[7:0]) : LCD module’s manufacturer ID.
    //                              The 3rd parameter(ID2[7:0]) : LCD module / driver version ID.
    //                              The 4th parameter(ID3[7:0]) : LCD module / driver ID.
    int ret ;
    static const uint8_t RDDIDIFReadCmd = ILI9341_RDDID;
    spiReadData = (uint8_t*)malloc(4 * sizeof(uint8_t));
    ret = spi_read (fb, spiReadData, RDDIDIFReadCmd, sizeof(spiReadData));
    if (ret == 0) {
        Log_Debug("INFO : SPI READ of RDDID 04h is %x\n", spiReadData);
    }
    else {
        Log_Debug("ERROR : SPI READ of RDDID 04h Failed\n");
    }
    return ExitCode_Success;
}

/*
static ExitCode ReadRDDIDIF_all(void) {
    static const uint8_t RDDIDIFReadCmd = ILI9341_RDDID;
    //static const uint8_t NOPCmd = ILI9341_NOP;
    uint8_t ili_readData[4];
    // Read register value using AppLibs combination read and write API
    static const size_t transferCount = 5;
    SPIMaster_Transfer transfers[transferCount];
    //uint8_t actualWhoAmIMultipleTransfers;

    int result = SPIMaster_InitTransfers(transfers, transferCount);
    if (result != 0) {
        Log_Debug("ERROR : SPIMaster_InitTransfers : %d",result);
        return ExitCode_ReadWhoAmI_InitTransfers;
    }

    transfers[0].flags = SPI_TransferFlags_Write;
    transfers[0].writeData = &RDDIDIFReadCmd;
    transfers[0].length = sizeof(RDDIDIFReadCmd);

    transfers[1].flags = SPI_TransferFlags_Read;
    transfers[1].readData = &ili_readData[0];
    transfers[1].length = sizeof(uint8_t);

    transfers[2].flags = SPI_TransferFlags_Read;
    transfers[2].readData = &ili_readData[1];
    transfers[2].length = sizeof(uint8_t);

    transfers[3].flags = SPI_TransferFlags_Read;
    transfers[3].readData = &ili_readData[2];
    transfers[3].length = sizeof(uint8_t);

    transfers[4].flags = SPI_TransferFlags_Read;
    transfers[4].readData = &ili_readData[3];
    transfers[4].length = sizeof(uint8_t);
    
    ssize_t transferredBytes = SPIMaster_TransferSequential(spiFd, transfers, transferCount);
    //if (!CheckTransferSize("SPIMaster_TransferSequential (CTRL3_C)",
    //    sizeof(actualWhoAmIMultipleTransfers) + sizeof(whoAmIRegIdReadCmd),
    //    transferredBytes)) {
    //   return ExitCode_ReadWhoAmI_TransferSequential;
    }
    Log_Debug("INFO: Data is%d,%d,%d,%d (SPIMaster_TransferSequential)\n",
        ili_readData[0], ili_readData[1], ili_readData[2], ili_readData[3] );
    //if (actualWhoAmIMultipleTransfers != expectedWhoAmI) {
    //    Log_Debug("ERROR: Unexpected WHO_AM_I value.\n");
    //    return ExitCode_ReadWhoAmI_TransferSequentialWrongWhoAmI;
    }
    
    // write() then read() does not work for this peripheral. Since that involves two
    // separate driver-level operations, the CS line is deasserted between the write()
    // and read(), and the peripheral loses state about the selected register.
    return ExitCode_Success;
}
*/

/**
  * @brief  Initialize ili9341 Driver Parameters and makes device Ready to send data.
  * @param  None.
  * @retval None.
  */
extern uint8_t ili9341_param_init(int fd) {
    int ret; 
    int ll = -1;
    int ctr = 0;
    //Soft Reset ILI9321 Driver
    ret = spi_write(fd, ILI9341_COMMAND, ILI9341_SWRESET_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : ILI9341_SWRESET_Cmd : Bytes Tfr %d\n", ret);
    msleep(1000);

    // Power_Control_A _Single Istr
    ret = spi_write(fd, ILI9341_COMMAND, Power_Control_A_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Power_Control_A_Cmd : Bytes Tfr %d\n", ret);
    ll = sizeof(Power_Control_A); ret = 0;
    while (ll != 0) {
        ret += spi_write(fd, ILI9341_DATA, Power_Control_A[ctr], 1);
        ctr++;
        --ll;
    }
    Log_Debug("INFO : ili9341_param_init : Power_Control_A : Bytes Tfr %d\n", ret);
    ret = -1; ctr = 0;

    // Power_Control_B _Single Istr
    ret = spi_write(fd, ILI9341_COMMAND, Power_Control_B_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Power_Control_B_Cmd : Bytes Tfr %d\n", ret);
    ll = sizeof(Power_Control_B); ret = 0;
    while (ll != 0) {
        ret += spi_write(fd, ILI9341_DATA, Power_Control_B[ctr], 1);
        ctr++;
        --ll;
    }
    Log_Debug("INFO : ili9341_param_init : Power_Control_B : Bytes Tfr %d\n", ret);
    ret = -1; ctr = 0;
   
    // Driver_Timing_Control_A
    ret = spi_write(fd, ILI9341_COMMAND, Driver_Timing_Control_A_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Driver_Timing_Control_A_Cmd : Bytes Tfr %d\n", ret);
    ll = sizeof(Driver_Timing_Control_A); ret = 0;
    while (ll != 0) {
        ret += spi_write(fd, ILI9341_DATA, Driver_Timing_Control_A[ctr], 1);
        ctr++;
        --ll;
    }
    Log_Debug("INFO : ili9341_param_init : Driver_Timing_Control_A : Bytes Tfr %d\n", ret);
    ret = -1; ctr = 0;

    // Driver_Timing_Control_B
    ret = spi_write(fd, ILI9341_COMMAND, Driver_Timing_Control_B_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Driver_Timing_Control_B_Cmd : Bytes Tfr %d\n", ret);
    ll = sizeof(Driver_Timing_Control_B); ret = 0;
    while (ll != 0) {
        ret += spi_write(fd, ILI9341_DATA, Driver_Timing_Control_B[ctr], 1);
        ctr++;
        --ll;
    }
    Log_Debug("INFO : ili9341_param_init : Driver_Timing_Control_B : Bytes Tfr %d\n", ret);
    ret = -1; ctr = 0;

    // Power_On_Sequence_Control
    ret = spi_write(fd, ILI9341_COMMAND, Power_On_Sequence_Control_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Power_On_Sequence_Control_Cmd : Bytes Tfr %d\n", ret);
    ll = sizeof(Power_On_Sequence_Control); ret = 0;
    while (ll != 0) {
        ret += spi_write(fd, ILI9341_DATA, Power_On_Sequence_Control[ctr], 1);
        ctr++;
        --ll;
    }
    Log_Debug("INFO : ili9341_param_init : Power_On_Sequence_Control : Bytes Tfr %d\n", ret);
    ret = -1; ctr = 0;

    // Pump_Ratio_Control
    ret = spi_write(fd, ILI9341_COMMAND, Pump_Ratio_Control_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Pump_Ratio_Control_Cmd : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, Pump_Ratio_Control, 1);
    Log_Debug("INFO : ili9341_param_init : Pump_Ratio_Control : Bytes Tfr %d\n", ret);

    // Power_Control_VRH5_0
    ret = spi_write(fd, ILI9341_COMMAND, Power_Control_VRH_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Power_Control_VRH5_0_Cmd : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, Power_Control_VRH5_0, 1);
    Log_Debug("INFO : ili9341_param_init : Power_Control_VRH5_0 : Bytes Tfr %d\n", ret);

    // Power_Control_SAP2_0_BT3_0
    ret = spi_write(fd, ILI9341_COMMAND, Power_Control_SAP_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Power_Control_SAP2_0_BT3_0 : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, Power_Control_SAP2_0_BT3_0, 1);
    Log_Debug("INFO : ili9341_param_init : Power_Control_SAP2_0_BT3_0 : Bytes Tfr %d\n", ret);

    // VCM_Control
    ret = spi_write(fd, ILI9341_COMMAND, VCM_Control_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : VCM_Control_Cmd : Bytes Tfr %d\n", ret);
    ll = sizeof(VCM_Control); ret = 0;
    while (ll != 0) {
        ret += spi_write(fd, ILI9341_DATA, VCM_Control[ctr], 1);
        ctr++;
        --ll;
    }
    Log_Debug("INFO : ili9341_param_init : VCM_Control : Bytes Tfr %d\n", ret);
    ret = -1; ctr = 0;

    // VCM_Control_2
    ret = spi_write(fd, ILI9341_COMMAND, VCM_Control_2_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : VCM_Control_2 : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, VCM_Control_2, 1);
    Log_Debug("INFO : ili9341_param_init : VCM_Control_2 : Bytes Tfr %d\n", ret);

    // Memory_Access_Control
    ret = spi_write(fd, ILI9341_COMMAND, Memory_Access_Control_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Memory_Access_Control_Cmd : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, Memory_Access_Control, 1);
    Log_Debug("INFO : ili9341_param_init : Memory_Access_Control : Bytes Tfr %d\n", ret);

    // Pixel_Format
    ret = spi_write(fd, ILI9341_COMMAND, Pixel_Format_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init :Pixel_Format_Cmd : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, Pixel_Format, 1);
    Log_Debug("INFO : ili9341_param_init : Pixel_Format : Bytes Tfr %d\n", ret);

    // Frame_Ratio_Control
    ret = spi_write(fd, ILI9341_COMMAND, Frame_Ratio_Control_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Frame_Ratio_Control_Cmd : Bytes Tfr %d\n", ret);
    ll = sizeof(Frame_Ratio_Control); ret = 0;
    while (ll != 0) {
        ret += spi_write(fd, ILI9341_DATA, Frame_Ratio_Control[ctr], 1);
        ctr++;
        --ll;
    }
    Log_Debug("INFO : ili9341_param_init : Frame_Ratio_Control : Bytes Tfr %d\n", ret);
    ret = -1; ctr = 0;

    // Display_Function_Control
    ret = spi_write(fd, ILI9341_COMMAND, Display_Function_Control_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Display_Function_Control_Cmd : Bytes Tfr %d\n", ret);
    ll = sizeof(Display_Function_Control); ret = 0;
    while (ll != 0) {
        ret += spi_write(fd, ILI9341_DATA, Display_Function_Control[ctr], 1);
        ctr++;
        --ll;
    }
    Log_Debug("INFO : ili9341_param_init : Display_Function_Control : Bytes Tfr %d\n", ret);
    ret = -1; ctr = 0;

    //  Gamma_Function_Disable
    ret = spi_write(fd, ILI9341_COMMAND, Gamma_Function_Disable_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init :  Gamma_Function_Disable_Cmd : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, Gamma_Function_Disable, 1);
    Log_Debug("INFO : ili9341_param_init : Gamma_Function_Disable : Bytes Tfr %d\n", ret);

    // Gamma_Curve_Select
    ret = spi_write(fd, ILI9341_COMMAND, Gamma_Curve_Select_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Gamma_Curve_Select_Cmd : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, Gamma_Curve_Select, 1);
    Log_Debug("INFO : ili9341_param_init : Gamma_Curve_Select : Bytes Tfr %d\n", ret);

    // Positive_Gamma_Correction
    ret = spi_write(fd, ILI9341_COMMAND, Positive_Gamma_Correction_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Positive_Gamma_Correction_Cmd : Bytes Tfr %d\n", ret);
    ll = sizeof(Positive_Gamma_Correction); ret = 0;
    while (ll != 0) {
        ret += spi_write(fd, ILI9341_DATA, Positive_Gamma_Correction[ctr], 1);
        ctr++;
        --ll;
    }
    Log_Debug("INFO : ili9341_param_init : Positive_Gamma_Correction : Bytes Tfr %d\n", ret);
    ret = -1; ctr = 0;

    // Negative_Gamma_Correction
    ret = spi_write(fd, ILI9341_COMMAND, Negative_Gamma_Correction_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init :Negative_Gamma_Correction_Cmd : Bytes Tfr %d\n", ret);
    ll = sizeof(Negative_Gamma_Correction); ret = 0;
    while (ll != 0) {
        ret += spi_write(fd, ILI9341_DATA, Negative_Gamma_Correction[ctr], 1);
        ctr++;
        --ll;
    }
    Log_Debug("INFO : ili9341_param_init : Negative_Gamma_Correction : Bytes Tfr %d\n", ret);
    ret = -1; ctr = 0;

    // Exit_Sleep
    ret = spi_write(fd, ILI9341_COMMAND, Exit_Sleep_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Exit_Sleep_Cmd : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, Exit_Sleep, 1);
    Log_Debug("INFO : ili9341_param_init : Exit_Sleep : Bytes Tfr %d\n", ret);
    msleep(200);

    // Turn_On_Display
    ret = spi_write(fd, ILI9341_COMMAND, Turn_On_Display_Cmd, 1);
    Log_Debug("INFO : ili9341_param_init : Turn_On_Display_Cmd : Bytes Tfr %d\n", ret);
    ret = spi_write(fd, ILI9341_DATA, Turn_On_Display, 1);
    Log_Debug("INFO : ili9341_param_init : Turn_On_Display : Bytes Tfr %d\n", ret);

    if (ret < 0) {
        Log_Debug("ERROR : ili9341_param_init : Negitive Bytes Tfr %d\n", ret);
        return ExitCode_Init_InitConfig;
    }
    return ExitCode_Success;  
}

/**
  * @brief  Initialize ili9341 SPI ONLY I/F.
  * @param  None.
  * @retval None.
  */
extern uint8_t ili9341_gpio_setup(void) {
    int ret;
    // initialiazing Extra Pins required for Display operation.
    rstfp = GPIO_OpenAsOutput(RST_PIN, GPIO_OutputMode_PushPull, GPIO_Value_High);
    dcfp = GPIO_OpenAsOutput(DC_PIN, GPIO_OutputMode_PushPull, GPIO_Value_High);
    ledfp = GPIO_OpenAsOutput(LED_PIN, GPIO_OutputMode_PushPull, GPIO_Value_High);
    if (rstfp == -1 || dcfp == -1 || ledfp == -1) {
        return ExitCode_Gpio_Open;
    }

    // setting d/c to d mode
    ret = GPIO_SetValue(dcfp, GPIO_Value_High);
    if (ret != 0) {
        Log_Debug("ERROR: ili9341_gpio_setup :Could not set dcfp gpio\n");
        return ExitCode_Gpio_Open;
    }

    // Enabling LED
    ret = GPIO_SetValue(ledfp, GPIO_Value_High);
    if (ret != 0) {
        Log_Debug("ERROR: ili9341_gpio_setup :Could not set ledfp gpio\n");
        return ExitCode_Gpio_Open;
    }

    //resetting the Display 
    ret = ili9341_reset(rstfp);
    if (ret != 0) {
        Log_Debug("ERROR: ili9341_gpio_setup : Could not RESET THE MCU\n");
        return ExitCode_Gpio_Open;
    }

    return ExitCode_Success;
}

/**
  * @brief  Initialize ili9341 SPI ONLY I/F.
  * @param  None.
  * @retval None.
  */
extern int ili9341_spi_setup(int fd) {
    int ret = SPIMaster_InitConfig(&config);
    if (ret != 0) {
        Log_Debug("ERROR: SPIMaster_InitConfig = %d\n", ret);
        return ExitCode_Init_InitConfig;
    }
    config.csPolarity = SPI_ChipSelectPolarity_ActiveLow;
    fd = SPIMaster_Open(ILI9341_SK_ISU1_SPI, ILI9341_SPI_CS, &config);
    if (fd < 0) {
        Log_Debug("ERROR: SPIMaster_Open : %d\n", fd);
        return ExitCode_Init_OpenSpiMaster;
    }

    int result = SPIMaster_SetBusSpeed(fd, 400000);
    if (result != 0) {
        Log_Debug("ERROR: SPIMaster_SetBusSpeed: %d\n", result);
        return ExitCode_Init_SetBusSpeed;
    }

    result = SPIMaster_SetMode(fd, SPI_Mode_3);
    if (result != 0) {
        Log_Debug("ERROR: SPIMaster_SetMode: %d\n", result);
        return ExitCode_Init_SetMode;
    }
    return fd;
}


/**
  * @brief  Closes File Discripter.
  * @param  None.
  * @retval None.
  */
extern uint8_t ili9341_close(int fd, const char* fdName) {
    int result;
    if (fd >= 0) {
        result = close(fd);
        if (result != 0) {
            Log_Debug("ERROR: Could not close spifd : %s.\n", fdName);
        }
    }
    result = close(dcfp);
    if (result != 0) {
        Log_Debug("ERROR: Could not close dcfp : %s.\n", fdName);
    }
    result = close(rstfp);
    if (result != 0) {
        Log_Debug("ERROR: Could not close rstfp : %s.\n", fdName);
    }
    result = close(ledfp);
    if (result != 0) {
        Log_Debug("ERROR: Could not close ledfp : %s.\n", fdName);
    }
}

/**
  * @brief  Initialize ili9341.
  * @param  None.
  * @retval None.
  */

extern int ili9341_init(int fd, const char *fdName) {
    int err;
    err = ili9341_gpio_setup();
    if (err != 0) {
        Log_Debug("ERROR: Could not open gpios fd : %s.\n", fdName);
    }
    
    fd = ili9341_spi_setup(fd);
    Log_Debug("INFO : spi_fd given a Fd value of %d : %s.\n",fd , fdName);

    err = ili9341_param_init(fd);
    if (err != 0) {
        Log_Debug("ERROR: Failed to init LCD Parameters : %s.\n", fdName);
    }

    //Setting Orientation
    ili9341_orientation(ILI9341_ORIENT_LANDSCAPE);

    ExitCode localExitCode = ReadRDDIDIF_ID2(fd);
    if (localExitCode != ExitCode_Success) {
        localExitCode = -1;//ResetAndSetSampleRange();
    }

    return fd;
}

extern int ili9341_reset_CS(int fd) {
    int err;
    ili9341_close(ledfp, "led");
    Log_Debug("ERROR: ili9341_reset_CS : closing ledfp");
    ledfp = GPIO_OpenAsOutput(LED_PIN, GPIO_OutputMode_PushPull, GPIO_Value_High);
    Log_Debug("ERROR: ili9341_reset_CS : setting ledfp %d", ledfp);
    err = GPIO_SetValue(ledfp, GPIO_Value_Low);
    ili9341_close(fd, "spi");
    msleep(500);
    ili9341_reset(rstfp);
    GPIO_SetValue(ledfp, GPIO_Value_High);

    fd = ili9341_spi_setup(fd);
    Log_Debug("INFO : spi_fd given a Fd value of %d .\n", fd);

    return fd;
}
