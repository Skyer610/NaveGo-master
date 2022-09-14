LI = length(ekinox_imu.t)
j=1
for i=1:LI
vx=odo.vel*cos(ekinox_imu.yaw)