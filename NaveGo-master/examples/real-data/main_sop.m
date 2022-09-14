clc
clear
simu_sop;
%nav_ekinox = ins_sop3_odo(ekinox_imu, ekinox_gnss,sop,odo);
%nav_ekinox = ins_gnss_sop3(ekinox_imu, ekinox_gnss,sop);
nav_ekinox = ins_sop3(ekinox_imu, ekinox_gnss,sop);
figure;
plot(ref.lon.*R2D, ref.lat.*R2D, '--k');
hold on
plot(nav_ekinox.lon.*R2D, nav_ekinox.lat.*R2D, 'Color', blue, 'LineWidth', lw);
hold off
n_ref = size(ref.t,1);
n_nav = size(nav_ekinox.t,1);
rmse = zeros(n_ref,4);
[RM,RN] = radius(ekinox_gnss.lat(1));
lontometer = RM + ekinox_gnss.h(1);
lattometer = (RN + ekinox_gnss.h(1)) * cos(ekinox_gnss.lat(1));
rmse_sum = 0;
rmse_lat = 0;
rmse_lon = 0;
rmse_h = 0;
for i=1:n_ref-2
    rmse(i,1) = sqrt( (lontometer*(ref.lon(i,1)-nav_ekinox.lon(i*200,1)))^2 + ( lattometer *(ref.lat(i,1)-nav_ekinox.lat(i*200,1)))^2 + (ref.h(i,1)-nav_ekinox.h(i*200,1))^2);
    rmse(i,2) = abs(lontometer*(ref.lon(i,1)-nav_ekinox.lon(i*200,1)));
    rmse(i,3) = abs(lattometer *(ref.lat(i,1)-nav_ekinox.lat(i*200,1)));
    rmse(i,4) = abs(ref.h(i,1)-nav_ekinox.h(i*200,1));
    rmse_lat = rmse_lat+rmse(i,3);
    rmse_lon = rmse_lon+rmse(i,2);
    rmse_h = rmse_h+rmse(i,4);
    rmse_sum =rmse_sum+rmse(i,1);

end
rmse = rmse_sum/n_ref
rmse_lat = rmse_lat/n_ref
rmse_lon = rmse_lon/n_ref
rmse_h = rmse_h/n_ref