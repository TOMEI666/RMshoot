export LD_LIBRARY_PATH=/home/sakanna/sp_vision_25/io/hikrobot/lib/amd64:/home/sakanna/sp_vision_25/io/hikrobot/lib/arm64:$LD_LIBRARY_PATH
sleep 5
cd ~/sp_vision_25/
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./watchdog.sh"
