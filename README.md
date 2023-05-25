### Loki - Blue

Name: Yongkang Sun, Haoming Shi  
Student ID: 45738050, 46055400

Design Task:  
The project is aim to create a network of atleast 2 air quality 
monitoring sensors using the Thingy:52 or ESP32 or STM32 Discovery.
and the SEN54. The stations should communicate with each other and a
base node via a Bluetooth Mesh connection. Mesh network connection
must be shown to work.

Folder structure:
- loki-blue
    - provisioner
        - src
            - main.c
        - prj.conf
        - CMakeLists.txt
    - base
        - src
            - main.c
        - prj.conf
        - CMakeLists.txt
    - ws
        - src
            - main.c
        - prj.conf
        - CMakeLists.txt
    - dv
        - dataviewer.py

Build Instructions:  
ws is for weather stations (Thingy:52), the maximum number of weather station is 2.  
Provisioner device is esp32c3 board and base node is esp32 as well, one of each.  
After build, the provision will add weather stations and base node into mesh network  
automatically. To see the result on web, execute dataviewer.py to upload the data.  

Schematics References:  
[esp32c3_devkitm datasheet](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/hw-reference/esp32c3/user-guide-devkitm-1.html)
[Thingy52 tutorial](https://uqiotstudio.github.io/csse4011_zephyr_guides/Boards/BRD.2-Thingy52/)