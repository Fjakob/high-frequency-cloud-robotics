clear;
close all;

run('../build/DATA.m')

index = DATAm(:,1);
time_cloud = DATAm(:,2);
time_robot = DATAm(:,3);
q_robot = DATAm(:,4:10);
dq_robot = DATAm(:,11:17);
q_cloud = DATAm(:,18:24);
dq_cloud = DATAm(:,25:31);
tau_ext_array = DATAm(:,32:38);
f_ext_meas = DATAm(:,39:44);
f_ext_meas_filtered = DATAm(:,45:50);
f_ext_kathib = DATAm(:,51:56);
f_des_2 = DATAm(:,57:59);
error_array = DATAm(:,60:65);
p_des = DATAm(:,66:68);
dx_des = DATAm(:,69:74);
position_cloud = DATAm(:,75:77);
position_robot = DATAm(:,78:80);
velocity_cloud = DATAm(:,81:83);
velocity_robot = DATAm(:,84:86);
E_cloud_in = DATAm(:,87);
E_cloud_out = DATAm(:,88);
E_robot_in = DATAm(:,89);
E_diss = DATAm(:,90);
beta = DATAm(:,91);
tau_com_cloud = DATAm(:,92:98);

figure()
plot(beta)
grid on
title('beta')



figure()
for idx=1:7
    subplot(3,3,idx)
    plot(q_robot(1:12000,idx),'DisplayName','robot')
    hold on
    grid on
    plot(q_cloud(2:12000,idx),'DisplayName','cloud')
    legend('show')
end
sgtitle('joint position drift')

figure()
plot(p_des(1:12000,1), p_des(1:12000,2), 'r--', 'DisplayName','p des');
hold on
plot(position_robot(1:12000,1), position_robot(1:12000,2), 'b', 'DisplayName','p robot');
plot(position_cloud(1:12000,1), position_cloud(1:12000,2), 'g', 'DisplayName','p cloud');
grid on;
legend('show')


