#!/bin/bash
echo "Starting server..."
sleep 5

#xset s noblank
#xset s off

#/home/omsi/smart_egg_drop/index.html 
sudo python3 /home/omsi/smart_egg_drop/server.py &
sleep 5
/usr/bin/chromium-browser --noerrdialogs --disable-infobars --kiosk http://localhost:8000/smart_egg_drop/ &

