#  Copyright (c) Microsoft Corporation. All rights reserved.
#  Licensed under the MIT License.

cmake_minimum_required (VERSION 3.8)
project (AzSphere_ILI9341_Test_HighLevelCore C)

# Create executable
add_executable (${PROJECT_NAME} main.c ILI9341_AzSphere_driver.c)
target_link_libraries (${PROJECT_NAME} applibs pthread gcc_s c)

# Add MakeImage post-build command
include (${AZURE_SPHERE_MAKE_IMAGE_FILE})
