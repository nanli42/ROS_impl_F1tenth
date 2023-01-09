echo "ego: x - " $1 " y - " $2 " yaw - " $3 
echo "opp: x - " $4 " y - " $5 " yaw - " $6
rostopic pub /reset_signal std_msgs/Float64MultiArray "{data:[$1,$2,$3,$4,$5,$6]}"
