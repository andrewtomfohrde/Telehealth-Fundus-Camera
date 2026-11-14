lxterminal -e "ssh -R 8082:127.0.0.1:22 \
 -R 5000:127.0.0.1:5000\
 langl261@csel-kh1250-13.cselabs.umn.edu"

lxterminal -e "sudo python3 remote_motor_stream.py"


