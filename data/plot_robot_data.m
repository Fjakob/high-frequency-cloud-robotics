addpath('../build');
addpath('../data');

run('robot_data.m');

idx = DATAm(:,1);
time_local = DATAm(1:12000,2);
time_remote = DATAm(:,3);
q_robot = DATAm(:,4:10);
q_cloud = DATAm(:,11:17);
dq_robot = DATAm(:,18:24);
dq_cloud = DATAm(:,25:31);
tau_ext_robot = DATAm(:,32:38);
f_ext_robot = DATAm(:,39:44);
tau_cloud_delay = DATAm(:,45:51);
f_cloud_delay = DATAm(:,52:57);
E_in_r = DATAm(:,58);
E_out_r = DATAm(:,59);
E_in_c_delay = DATAm(:,60);
E_diss_r = DATAm(:,61);
alpha = DATAm(1:12000,62);
tau_robot = DATAm(:,63:69);
x_robot = DATAm(1:12000,70:72);

figure()
plot(time_local, x_robot(:,1), 'DisplayName','x')
hold on
grid on
plot(time_local, x_robot(:,2), 'DisplayName','y')
plot(time_local, x_robot(:,3), 'DisplayName','z')
legend('show')
xlabel('Time')
ylabel('Cartesian position')


figure()
plot(x_robot(:,1), x_robot(:,2), 'DisplayName','cirle')
hold on
grid on
legend('show')
xlabel('x')
ylabel('y')

figure()
plot(time_local, alpha)
hold on
grid on
legend('show')
xlabel('Time')
ylabel('\alpha')
alpha_max = max(alpha)
alpha_mean = mean(alpha)