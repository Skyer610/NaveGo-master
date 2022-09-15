%% REFERENCE DATA

% Reference dataset was obtained by processing Ekinox IMU and Ekinox GNSS 
% with tighly-coupled integration by Inertial Explorer software package.
clc
fprintf('NaveGo: loading reference data... \n')
D2R = (pi/180);     % degrees to radians

load ("ref.mat")

%% EKINOX IMU 

fprintf('NaveGo: loading Ekinox IMU data... \n')

load ("ekinox_imu.mat")

%% EKINOX GNSS 

fprintf('NaveGo: loading Ekinox GNSS data... \n')

load ("ekinox_gnss.mat")

%% figure
R2D = (180/pi);     % radians to degrees

plot_parameters;

figure;
plot(ref.lon.*R2D, ref.lat.*R2D, 'b');
hold on;
plot(7.646, 45.045, 'or', 'MarkerSize', ms, 'LineWidth', lw)
plot(7.654, 45.035, 'or', 'MarkerSize', ms, 'LineWidth', lw)
plot(7.65,  45.06, 'or', 'MarkerSize', ms, 'LineWidth', lw)

%l1 = legend('REF', 'INS/GNSS', 'Starting point', 'Location', 'SouthEast');

psop =[7.646,45.045,330;
    7.654,45.035,330;
    7.65,45.06,330]; 


psop = psop * D2R;
 LG = length(ekinox_gnss.t);
 LS = LG*5;
 LO = LG/5;
 refs = zeros(LS,3);
for i = 1:LG-1
    for j = 0:4
        refs(i*5+j,1) = 0.2*((5-j)*ekinox_gnss.lon(i,1) + j*ekinox_gnss.lon(i+1,1));
         refs(i*5+j,2) = 0.2*((5-j)*ekinox_gnss.lat(i,1) + j*ekinox_gnss.lat(i+1,1));
          refs(i*5+j,3) = 0.2*((5-j)*ekinox_gnss.h(i,1) + j*ekinox_gnss.h(i+1,1));
    end
end

[RM,RN] = radius(ekinox_gnss.lat(1));
lattometer = RM + ekinox_gnss.h(1);
lontometer = (RN + ekinox_gnss.h(1)) * cos(ekinox_gnss.lat(1));
for i = 1:LS
    sop.t(i,1) = ref.t(1)+0.1*i;
    sop.lon(i,1:3) = psop(1:3,1)';
    sop.lat(i,1:3) = psop(1:3,2)';
    sop.h(i,1:3) = psop(1:3,3)';
    sop.dis(i,1) = sqrt( (lontometer*(refs(i,1)-sop.lon(i,1)))^2 + ( lattometer *(refs(i,2)-sop.lat(i,1)))^2 + (refs(i,3)-sop.h(i,1))^2)+10*rand(1);
    sop.dis(i,2) = sqrt( (lontometer*(refs(i,1)-sop.lon(i,2)))^2 + ( lattometer *(refs(i,2)-sop.lat(i,2)))^2 + (refs(i,3)-sop.h(i,2))^2)+10*rand(1);
    sop.dis(i,3) = sqrt( (lontometer*(refs(i,1)-sop.lon(i,3)))^2 + ( lattometer *(refs(i,2)-sop.lat(i,3)))^2 + (refs(i,3)-sop.h(i,3))^2)+10*rand(1);
end
sop.std = [10 10 10];
sop.eps = 0.0025;
sop.dis(1:4,:) = sop.dis(5:8,:) ;
for j =1:LO
 odo.t(j,1) = ekinox_gnss.t((j-1)*5+1);
 odo.vel(j,1) = sqrt(ekinox_gnss.vel((j-1)*5+1,1)^2 + ekinox_gnss.vel((j-1)*5+1,2)^2 + ekinox_gnss.vel((j-1)*5+1,3)^2);
 odo.vel3(j,:)= ekinox_gnss.vel((j-1)*5+1,:);
end
odo.stdv = ekinox_gnss.stdv;
odo.freq = 1;
odo.larm = ekinox_gnss.larm;