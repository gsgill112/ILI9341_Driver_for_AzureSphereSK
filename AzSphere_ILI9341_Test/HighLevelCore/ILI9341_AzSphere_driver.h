//This Driver is for ILI9341 Display Driver commonly found on 2.2" generic Display Module
//Ver 0.1


#ifndef AzSphere_ILI9341_DRV_H
#define AzSphere_ILI9341_DRV_H

//Global Includes
#include <applibs/spi.h>
#include <applibs/gpio.h>
#include <applibs/log.h>

//Definitions for external Pins based on Click Slot Configured
#if CLICK_SLOT == 1 
    #define LED_PIN AVNET_MT3620_SK_GPIO2
    #define DC_PIN  AVNET_MT3620_SK_GPIO42
    #define RST_PIN AVNET_MT3620_SK_GPIO16
//    Log_Debug("Configured for Mk Bus CLICK SLOT #1");
#else 
    #define LED_PIN AVNET_MT3620_SK_GPIO2
    #define DC_PIN  AVNET_MT3620_SK_GPIO43
    #define RST_PIN AVNET_MT3620_SK_GPIO17
//    Log_Debug("Configured for Mk Bus CLICK SLOT #1");
#endif

//Defined Variables 
extern int rstfp, dcfp, ledfp;
int max_x, max_y;

#define ILI9341_COMMAND 1 //Sending if command /Data 
#define ILI9341_DATA 0 //Sending if command /Data 

//Definition for ILI9341Driver 
#define ILI9341_ORIENT_POTRAIT 0 ///Sets Potrait Orient 
#define ILI9341_ORIENT_LANDSCAPE 1  // Sets Landscape Orient
#define ILI9341_TFTWIDTH 240  ///< ILI9341 max TFT width
#define ILI9341_TFTHEIGHT 320 ///< ILI9341 max TFT height
#define BUFFER_SIZE ILI9341_TFTWIDTH*ILI9341_TFTHEIGHT/8

#define ILI9341_NOP 0x00     ///< No-op register
#define ILI9341_SWRESET 0x01 ///< Software reset register
#define ILI9341_RDDID 0x04   ///< Read display identification information
#define ILI9341_RDDST 0x09   ///< Read Display Status

#define ILI9341_SLPIN 0x10  ///< Enter Sleep Mode
#define ILI9341_SLPOUT 0x11 ///< Sleep Out
#define ILI9341_PTLON 0x12  ///< Partial Mode ON
#define ILI9341_NORON 0x13  ///< Normal Display Mode ON

#define ILI9341_RDMODE 0x0A     ///< Read Display Power Mode
#define ILI9341_RDMADCTL 0x0B   ///< Read Display MADCTL
#define ILI9341_RDPIXFMT 0x0C   ///< Read Display Pixel Format
#define ILI9341_RDIMGFMT 0x0D   ///< Read Display Image Format
#define ILI9341_RDSELFDIAG 0x0F ///< Read Display Self-Diagnostic Result

#define ILI9341_INVOFF 0x20   ///< Display Inversion OFF
#define ILI9341_INVON 0x21    ///< Display Inversion ON
#define ILI9341_GAMMASET 0x26 ///< Gamma Set
#define ILI9341_DISPOFF 0x28  ///< Display OFF
#define ILI9341_DISPON 0x29   ///< Display ON

#define ILI9341_CASET 0x2A ///< Column Address Set
#define ILI9341_PASET 0x2B ///< Page Address Set
#define ILI9341_RAMWR 0x2C ///< Memory Write
#define ILI9341_RAMRD 0x2E ///< Memory Read

#define ILI9341_PTLAR 0x30    ///< Partial Area
#define ILI9341_VSCRDEF 0x33  ///< Vertical Scrolling Definition
#define ILI9341_MADCTL 0x36   ///< Memory Access Control
#define ILI9341_VSCRSADD 0x37 ///< Vertical Scrolling Start Address
#define ILI9341_PIXFMT 0x3A   ///< COLMOD: Pixel Format Set

#define ILI9341_FRMCTR1                                                        \
  0xB1 ///< Frame Rate Control (In Normal Mode/Full Colors)
#define ILI9341_FRMCTR2 0xB2 ///< Frame Rate Control (In Idle Mode/8 colors)
#define ILI9341_FRMCTR3                                                        \
  0xB3 ///< Frame Rate control (In Partial Mode/Full Colors)
#define ILI9341_INVCTR 0xB4  ///< Display Inversion Control
#define ILI9341_DFUNCTR 0xB6 ///< Display Function Control

#define ILI9341_PWCTR1 0xC0 ///< Power Control 1
#define ILI9341_PWCTR2 0xC1 ///< Power Control 2
#define ILI9341_PWCTR3 0xC2 ///< Power Control 3
#define ILI9341_PWCTR4 0xC3 ///< Power Control 4
#define ILI9341_PWCTR5 0xC4 ///< Power Control 5
#define ILI9341_VMCTR1 0xC5 ///< VCOM Control 1
#define ILI9341_VMCTR2 0xC7 ///< VCOM Control 2

#define ILI9341_RDID1 0xDA ///< Read ID 1
#define ILI9341_RDID2 0xDB ///< Read ID 2
#define ILI9341_RDID3 0xDC ///< Read ID 3
#define ILI9341_RDID4 0xDD ///< Read ID 4

#define ILI9341_GMCTRP1 0xE0 ///< Positive Gamma Correction
#define ILI9341_GMCTRN1 0xE1 ///< Negative Gamma Correction

// Define Commands for Drivit initialiazation 
static const uint8_t ILI9341_SWRESET_Cmd = 0x01;
static const uint8_t  Power_Control_A_Cmd = 0xCB;
static const uint8_t  Power_Control_B_Cmd = 0xCF;
static const uint8_t  Driver_Timing_Control_A_Cmd = 0xE8;
static const uint8_t  Driver_Timing_Control_B_Cmd = 0xEA;
static const uint8_t  Power_On_Sequence_Control_Cmd = 0xED;
static const uint8_t  Pump_Ratio_Control_Cmd = 0xF7;
static const uint8_t  Power_Control_VRH_Cmd = 0xC0;
static const uint8_t  Power_Control_SAP_Cmd = 0xC1;
static const uint8_t  VCM_Control_Cmd = 0xC5;
static const uint8_t  VCM_Control_2_Cmd = 0xC7;
static const uint8_t  Memory_Access_Control_Cmd = 0x36;
static const uint8_t  Pixel_Format_Cmd = 0x3A;
static const uint8_t  Frame_Ratio_Control_Cmd = 0xB1;
static const uint8_t  Display_Function_Control_Cmd = 0xB6;
static const uint8_t  Gamma_Function_Disable_Cmd = 0xF2;
static const uint8_t  Gamma_Curve_Select_Cmd = 0x26;
static const uint8_t  Positive_Gamma_Correction_Cmd = 0xE0;
static const uint8_t  Negative_Gamma_Correction_Cmd = 0xE1;
static const uint8_t  Exit_Sleep_Cmd = 0x11;
static const uint8_t  Turn_On_Display_Cmd = 0x29;
static const uint8_t  Column_Address_Set_cmd = 0x2A;
static const uint8_t  Page_Address_Set_Cmd = 0x2B;
static const uint8_t  Memory_Write_Cmd = 0x2C;
//#define ILI9341_PWCTR6     0xFC

// Color definitions
#define ILI9341_BLACK 0x0000       ///<   0,   0,   0
#define ILI9341_NAVY 0x000F        ///<   0,   0, 123
#define ILI9341_DARKGREEN 0x03E0   ///<   0, 125,   0
#define ILI9341_DARKCYAN 0x03EF    ///<   0, 125, 123
#define ILI9341_MAROON 0x7800      ///< 123,   0,   0
#define ILI9341_PURPLE 0x780F      ///< 123,   0, 123
#define ILI9341_OLIVE 0x7BE0       ///< 123, 125,   0
#define ILI9341_LIGHTGREY 0xC618   ///< 198, 195, 198
#define ILI9341_DARKGREY 0x7BEF    ///< 123, 125, 123
#define ILI9341_BLUE 0x001F        ///<   0,   0, 255
#define ILI9341_GREEN 0x07E0       ///<   0, 255,   0
#define ILI9341_CYAN 0x07FF        ///<   0, 255, 255
#define ILI9341_RED 0xF800         ///< 255,   0,   0
#define ILI9341_MAGENTA 0xF81F     ///< 255,   0, 255
#define ILI9341_YELLOW 0xFFE0      ///< 255, 255,   0
#define ILI9341_WHITE 0xFFFF       ///< 255, 255, 255
#define ILI9341_ORANGE 0xFD20      ///< 255, 165,   0
#define ILI9341_GREENYELLOW 0xAFE5 ///< 173, 255,  41
#define ILI9341_PINK 0xFC18        ///< 255, 130, 198

#define DEFAULT_BGCOLOR ILI9341_BLACK
/**
  * @brief  Initialize ili9341.
  * @param  None.
  * @retval None.
  */
extern int ili9341_init(int , const char*);
int spi_write(int, uint16_t, uint8_t, int);
int ili9341_reset_CS(int);
uint8_t ili9341_reset(int);

/**
  * @brief  Draw a pixel at specified coordinates
  * @param  x: x coordinate
  * @param  y: y coordinate
  * @retval None.
  */
void ILI9341_Draw_Pixel(int, uint16_t, uint16_t, uint16_t);

/**
  * @brief  Draw a line
  * @param  x1: x coordinate of start point
  * @param  y1: y coordinate of start point
  * @param  x2: x coordinate of end point
  * @param  y2: y coordinate of end point
  * @retval None.
  */
extern void ili9341_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);

/**
  * @brief  Draw a vertical line
  * @param  x: x coordinate of start point
  * @param  y: y coordinate of start point
  * @param  length: length of the line
  * @retval None.
  */
extern void ili9341_draw_fast_Vline(uint8_t x, uint8_t y, uint8_t length, uint8_t color);

/**
  * @brief  Draw a horizontal line
  * @param  x: x coordinate of start point
  * @param  y: y coordinate of start point
  * @param  length: length of the line
  * @retval None.
  */
extern void ili9341_draw_fast_Hline(uint8_t x, uint8_t y, uint8_t length, uint8_t color);

/**
  * @brief  Draw a rectangle given start point, width and height
  * @param  x: x coordinate
  * @param  y: y coordinate
  * @param  width: rectangle width
  * @param  height: rectangle height
  * @retval None.
  */
extern void ili9341_draw_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);

/**
  * @brief  Draw a fill rectangle given start point, width and height
  * @param  x: x coordinate
  * @param  y: y coordinate
  * @param  width: rectangle width
  * @param  height: rectangle height
  * @retval None.
  */
extern void ili9341_draw_fill_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);

/**
  * @brief  Draw a rounded rectangle given start point, width and height
  * @param  x: x coordinate
  * @param  y: y coordinate
  * @param  width: rectangle width
  * @param  height: rectangle height
  * @param  radius: radius of rounded corner
  * @retval None.
  */
extern void ili9341_draw_round_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t radius, uint8_t color);

/**
  * @brief  Draw a fill rounded rectangle given start point, width and height
  * @param  x: x coordinate
  * @param  y: y coordinate
  * @param  width: rectangle width
  * @param  height: rectangle height
  * @param  radius: radius of rounded corner
  * @retval None.
  */
extern void ili9341_draw_fillround_Rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t radius, uint8_t color);

/**
  * @brief  Draw a circle
  * @param  x: x center coordinate
  * @param  y: y center coordinate
  * @param  radius: radius of circle
  * @retval None.
  */
extern void ili9341_draw_circle(int32_t x, int32_t y, int32_t radius, uint8_t color);

/**
  * @brief  Draw a fill circle
  * @param  x: x center coordinate
  * @param  y: y center coordinate
  * @param  radius: radius of circle
  * @retval None.
  */
extern void ili9341_draw_fill_circle(int32_t x, int32_t y, int32_t radius, uint8_t color);

/**
  * @brief  Draw a triangle
  * @param  x0: first point's x coordinate
  * @param  y0: first point's y coordinate
  * @param  x1: second point's x coordinate
  * @param  y1: second point's y coordinate
  * @param  x2: third point's x coordinate
  * @param  y2: third point's y coordinate
  * @retval None.
  */
extern void ili9341_draw_triangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);

/**
  * @brief  Draw a fill triangle
  * @param  x0: first point's x coordinate
  * @param  y0: first point's y coordinate
  * @param  x1: second point's x coordinate
  * @param  y1: second point's y coordinate
  * @param  x2: third point's x coordinate
  * @param  y2: third point's y coordinate
  * @retval None.
  */
extern void ili9341_draw_fill_triangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);

/**
  * @brief  Draw a string
  * @param  x: x coordinate of start point
  * @param  y: y coordinate of start point
  * @param  textptr: pointer
  * @param  size: scale
  * @retval None.
  */
extern void ili9341_draw_string(int32_t x, int32_t y, uint8_t* textptr, int32_t size, uint8_t color);

/**
  * @brief  Used to do round rectangles
  * @param  x0: x center coordinate
  * @param  y0: y center coordinate
  * @param  radius: radius
  * @param  cornername: corner to draw the semicircle
  * @retval None.
  */
void ili9341_draw_circle_helper(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t cornername, uint8_t color);

/**
  * @brief  Used to do fill round rectangles
  * @param  x0: x center coordinate
  * @param  y0: y center coordinate
  * @param  radius: radius
  * @param  cornername: corner to draw the semicircle
  * @param  delta:
  * @retval None.
  */
void ili9341_draw_fillcircle_helper(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t cornername, uint8_t delta, uint8_t color);

/**
  * @brief  Set the display upside up.
  * @retval None.
  */
extern void upside_up(void);

/**
  * @brief  Set the display upside down.
  * @retval None.
  */
extern void upside_down(void);

/**
  * @brief  Send OLED buffer to OLED RAM
  * @retval None.
  */
extern void ili9341_refresh(void);

/**
  * @brief  Draw a image in OLED buffer
  * @retval None.
  */
extern void ili9341_draw_img(uint8_t* ptr_img);

/**
  * @brief  Set all buffer's bytes to zero
  * @retval None.
  */
extern void clear_lcd_buffer(void);

/**
  * @brief  Set all buffer's bytes to 0xff
  * @retval None.
  */
extern void fill_lcd_buffer(void);

/**
  * @brief  Draw an arc given angles (This is jus a test function, not optimized)
  * @param x: x coordinate of the center
  * @param y: y coordinate of the center
  * @param radius: radius of arc
  * @param a0: start angle
  * @param a1: end angle
  * @retval None.
  */
extern void ili9341_draw_arc(int32_t x, int32_t y, int32_t radius, int32_t a0, int32_t a1, uint8_t color);

extern void msleep(unsigned int);

extern uint8_t ili9341_close(int , const char* );
/*
void ILI9341_SPI_Init(void);
void ILI9341_SPI_Send(unsigned char SPI_Data);
void ILI9341_Write_Command(uint8_t Command);
void ILI9341_Write_Data(uint8_t Data);
void ILI9341_Set_Address(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2);
void ILI9341_Reset(void);
void ILI9341_Set_Rotation(uint8_t Rotation);
void ILI9341_Enable(void);
void ILI9341_Init(void);
void ILI9341_Fill_Screen(uint16_t Colour);
void ILI9341_Draw_Colour(uint16_t Colour);
void ILI9341_Draw_Pixel(uint16_t X, uint16_t Y, uint16_t Colour);
void ILI9341_Draw_Colour_Burst(uint16_t Colour, uint32_t Size);


void ILI9341_Draw_Rectangle(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint16_t Colour);
void ILI9341_Draw_Horizontal_Line(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Colour);
void ILI9341_Draw_Vertical_Line(uint16_t X, uint16_t Y, uint16_t Height, uint16_t Colour);
*/
#endif