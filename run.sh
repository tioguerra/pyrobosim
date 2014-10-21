python biped_server.py $1 & python client_standstill.py 127.0.0.1 $1 $2; python client_bipedwalk.py 127.0.0.1 $1 $2

