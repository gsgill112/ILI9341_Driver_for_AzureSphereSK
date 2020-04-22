
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "applibs_versions.h"

#include <applibs/log.h>
#include <applibs/spi.h>
#include "hw/avnet_mt3620_sk.h"
#include "ILI9341_AzSphere_driver.h"

//Custom Defines
#define CLICK_SLOT 1 // Change to 2 if using slot 2

// File descriptors - initialized to invalid value
//connected to AVNET_MT3620_SK_ISU1_SPI
int spiFd = -1;


int main(void)
{
    // This minimal Azure Sphere app repeatedly prints "Tick" then "Tock" to the debug console.
    // Use this app to test that device and SDK installation succeeded that you can build,
    // deploy, and debug an app with Visual Studio, and that you can deploy an app over the air,
    // per the instructions here: https://docs.microsoft.com/azure-sphere/quickstarts/qs-overview
    //
    // It is NOT recommended to use this as a starting point for developing apps; instead use
    // the extensible samples here: https://github.com/Azure/azure-sphere-samples

    spiFd = ili9341_init(spiFd, "SPI MT3620_SK_ISU1_SPI");
    while (spiFd < 0) {
        Log_Debug("Error Occured : Code %d\n", spiFd);
    }

    uint16_t x = 1;
    //ILI9341_Fill_Screen(spiFd, ILI9341_MAROON);
    for (x = 1; x <= ILI9341_TFTWIDTH ; x++)
        ILI9341_Draw_Pixel(spiFd, x, (uint8_t)10, ILI9341_MAROON);

    int ret = ili9341_close(spiFd, "SPI MT3620_SK_ISU1_SPI");
    while (ret != 0) {
        Log_Debug("Error Occured : Code %d\n", ret);
    }
    
    while (true) {
        Log_Debug("High Level Core tick\n");
        msleep(1000);
        Log_Debug("High Level Core tock\n");
        msleep(1000);
    }
}
