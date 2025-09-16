# Setup Guide for RGB Display

### Step 1: Configure Arduino IDE
Go to **Tools** and set the following:
- **Board:** ESP32S3 Dev Module  
- **Flash Mode:** QIO 80MHz  
- **Flash Size:** 16MB  
- **PSRAM:** OPI PSRAM  

### Step 2: Install Required Libraries
Open the **Library Manager** and install:
- **LovyanGFX** (v1.1.12)  
- **lvgl** (v8.3.11)  

### Step 3: Configure lvgl
- Take the `lv_conf.h` file provided in this repo.  
- Paste it inside your **Arduino libraries** folder.  

### Step 4: Run the Program
Once everything is set up, you can upload and run the program.  



# Instructions

### Before Uploading the Code
- Change the Wi-Fi credentials according to the device.  
- Change the topic according to the device assembled.  

### After Uploading the Code
- Connect the display with the device and test it without full assembly.  
- Check whether the values are updated accordingly.  
