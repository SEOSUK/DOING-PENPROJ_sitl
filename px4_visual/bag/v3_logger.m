
clear all; close all; clc;

%%
csv_path = '1st.csv';

data = readtable(csv_path, 'VariableNamingRule', 'preserve');

% 3. 시간 벡터
time = data{:,1}*10^(-9);  % 첫 번째 열은 상대 시간(sec)
% 4. 데이터 행렬 (나머지 열들)
values = data{:,2:end};  % 각 column은 data[0], data[1], ...
values_size = size(values(:,1));

%% === Global Plot Settings (window + 0-based xtick labels) ===
x_start = 40;              % 왼쪽 경계 [s]
x_end   = 85;             % 오른쪽 경계 [s]
xtick_step = 20;           % xtick 간격 [s]
xtick_vals   = x_start:xtick_step:x_end;
xtick_labels = string(xtick_vals - x_start);  % 0부터 시작한 것처럼 표시
default_size = [100 100 1200 600];

%%
position                     =zeros(values_size(1),3);
desired_position             =zeros(values_size(1),3);

linear_velocity              =zeros(values_size(1),3);
desired_linear_velocity      =zeros(values_size(1),3);

attitude                     =zeros(values_size(1),3);
desired_attitude             =zeros(values_size(1),3);

angular_velocity             =zeros(values_size(1),3);
desired_angular_velocity     =zeros(values_size(1),3);

desired_force                =zeros(values_size(1),3);

desired_torque               =zeros(values_size(1),4);
torque_dhat                  =zeros(values_size(1),3);

individual_motor_thrust      =zeros(values_size(1),4);

servo_angle                  =zeros(values_size(1),5);
desired_servo_angle          =zeros(values_size(1),5);

acceleration                 =zeros(values_size(1),3);
desired_acceleration         =zeros(values_size(1),3);

present_com                  =zeros(values_size(1),3);
past_com                     =zeros(values_size(1),3);
com_tilde                    =zeros(values_size(1),3);
com_update                   =zeros(values_size(1),3);

PWM_cmd                      =zeros(values_size(1),8);

%% Main_Drone Data_logging
%------------------------------------------------------------%
position(:,1)                     =values(:,2);
position(:,2)                     =values(:,3);
position(:,3)                     =values(:,4);
desired_position(:,1)             =values(:,5);
desired_position(:,2)             =values(:,6);
desired_position(:,3)             =values(:,7);
%------------------------------------------------------------%
linear_velocity(:,1)              =values(:,8);
linear_velocity(:,2)              =values(:,9);
linear_velocity(:,3)              =values(:,10);
desired_linear_velocity(:,1)      =values(:,11);
desired_linear_velocity(:,2)      =values(:,12);
desired_linear_velocity(:,3)      =values(:,13);
%------------------------------------------------------------%
attitude(:,1)                     =values(:,14);
attitude(:,2)                     =values(:,15);
attitude(:,3)                     =values(:,16);
desired_attitude(:,1)             =values(:,17);
desired_attitude(:,2)             =values(:,18);
desired_attitude(:,3)             =values(:,19);
%------------------------------------------------------------%
angular_velocity(:,1)             =values(:,20);
angular_velocity(:,2)             =values(:,21);
angular_velocity(:,3)             =values(:,22);
desired_angular_velocity(:,1)     =values(:,23);
desired_angular_velocity(:,2)     =values(:,24);
desired_angular_velocity(:,3)     =values(:,25);
%------------------------------------------------------------%
desired_force(:,1)                =values(:,26);
desired_force(:,2)                =values(:,27);
desired_force(:,3)                =values(:,28);
%------------------------------------------------------------%
desired_torque(:,1)               =values(:,29);
desired_torque(:,2)               =values(:,30);
desired_torque(:,3)               =values(:,31);
desired_torque(:,4)               =values(:,32);
%------------------------------------------------------------%
torque_dhat(:,1)                  =values(:,33);
torque_dhat(:,2)                  =values(:,34);
torque_dhat(:,3)                  =values(:,35);
%------------------------------------------------------------%
individual_motor_thrust(:,1)      =values(:,36);
individual_motor_thrust(:,2)      =values(:,37);
individual_motor_thrust(:,3)      =values(:,38);
individual_motor_thrust(:,4)      =values(:,39);
%------------------------------------------------------------%
servo_angle(:,1)                  =values(:,40);
servo_angle(:,2)                  =values(:,41);
servo_angle(:,3)                  =values(:,42);
servo_angle(:,4)                  =values(:,43);
servo_angle(:,5)                  =values(:,44);
%------------------------------------------------------------%
desired_servo_angle(:,1)          =values(:,45);
desired_servo_angle(:,2)          =values(:,46);
desired_servo_angle(:,3)          =values(:,47);
desired_servo_angle(:,4)          =values(:,48);
desired_servo_angle(:,5)          =values(:,49);
%------------------------------------------------------------%
acceleration(:,1)                 =values(:,50);
acceleration(:,2)                 =values(:,51);
acceleration(:,3)                 =values(:,52);
desired_acceleration(:,1)         =values(:,53);
desired_acceleration(:,2)         =values(:,54);
desired_acceleration(:,3)         =values(:,55);
%------------------------------------------------------------%
present_com(:,1)                  =values(:,56);
present_com(:,2)                  =values(:,57);
present_com(:,3)                  =values(:,58);
%------------------------------------------------------------%
past_com(:,1)                     =values(:,59);
past_com(:,2)                     =values(:,60);
past_com(:,3)                     =values(:,61);
%------------------------------------------------------------%
com_tilde(:,1)                    =values(:,62);
com_tilde(:,2)                    =values(:,63);
com_tilde(:,3)                    =values(:,64);
%------------------------------------------------------------%
com_update(:,1)                   =values(:,65);
com_update(:,2)                   =values(:,66);
com_update(:,3)                   =values(:,67);
%------------------------------------------------------------%

%% Attitude
figure('Position',[100 100 600 600], 'Color','w');

ax1 = subplot(3,1,1);
plot(time,desired_attitude(:,1),'-','LineWidth',2.5); hold on
plot(time,attitude(:,1),'-','LineWidth',2.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
ylabel('$\phi$ [rad]','Interpreter','latex','FontSize',14);

ax2 = subplot(3,1,2);
plot(time,desired_attitude(:,2),'-','LineWidth',2.5); hold on
plot(time,attitude(:,2),'-','LineWidth',2.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',14);

ax3 = subplot(3,1,3);
plot(time,desired_attitude(:,3) - 0.0092,'-','LineWidth',2.5); hold on
plot(time,attitude(:,3) - 0.0092,'-','LineWidth',2.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('$\psi$ [rad]','Interpreter','latex','FontSize',14);

% === Y축 한번에 통일 ===
linkaxes([ax1 ax2],'y');          % y축 동기화
ylim(ax1,[-0.1 0.1]);           % ax1 기준으로 적용 (ax2에도 동기화)
ylim(ax3,[-0.1 0.1]);           % ax3는 독립으로 지정

%%  XYZ Position
figure('Position',[100 100 600 600], 'Color','w');

ax1 = subplot(3,1,1);
plot(time,desired_position(:,1),'-','LineWidth',2.5); hold on
plot(time,position(:,1),'LineWidth',2.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
ylabel('$x$ [m]','Interpreter','latex','FontSize',14);

ax2 = subplot(3,1,2);
plot(time,desired_position(:,2),'-','LineWidth',2.5); hold on
plot(time,position(:,2),'LineWidth',2.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
ylabel('$y$ [m]','Interpreter','latex','FontSize',14);

ax3 = subplot(3,1,3);
plot(time,desired_position(:,3),'-','LineWidth',2.5); hold on
plot(time,position(:,3),'LineWidth',2.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('$z$ [m]','Interpreter','latex','FontSize',14);

% === Y축 한번에 통일 ===
linkaxes([ax1 ax2],'y');        % y축 동기화
ylim(ax1,[-1 1]);               % ax1, ax2 범위 동기화
ylim(ax3,[-0.9 -0.4]);          % ax3는 독립


%% 3D Trajectory Plot (108~128s 구간만)
idx = (time >= x_start) & (time <= x_end);   % 선택 구간 인덱스

figure('Position',default_size,'Color','w')
plot3(desired_position(idx,1), desired_position(idx,2), desired_position(idx,3), ...
      'r-', 'LineWidth', 2.5); hold on
plot3(position(idx,1), position(idx,2), position(idx,3), ...
      'b-', 'LineWidth', 2.0);
grid on
xlabel('$\bf{x}$ \rm\bf{(m)}','Interpreter','latex','FontSize',15)
ylabel('$\bf{y}$ \rm\bf{(m)}','Interpreter','latex','FontSize',15)
zlabel('$\bf{z}$ \rm\bf{(m)}','Interpreter','latex','FontSize',15)
legend({'$^{G}\mathbf{p}^*_{i}$','$^{G}\hat{\mathbf{p}}_{i}$'}, ...
       'Interpreter','latex','Location','best','FontSize',15)
title('3D Global Position Trajectory','FontSize',15)
axis equal; view(3)

% === 축 여백(pad) 주기 — range() 없이 동작 ===
pad = 0.05; % 5% 여백

x_all = [position(idx,1);          desired_position(idx,1)];
y_all = [position(idx,2);          desired_position(idx,2)];
z_all = [position(idx,3);          desired_position(idx,3)];

xr = [min(x_all) max(x_all)]; dx = diff(xr); if dx==0, dx = max(1e-6, abs(xr(1))*0.1); end
yr = [min(y_all) max(y_all)]; dy = diff(yr); if dy==0, dy = max(1e-6, abs(yr(1))*0.1); end
zr = [min(z_all) max(z_all)]; dz = diff(zr); if dz==0, dz = max(1e-6, abs(zr(1))*0.1); end

xlim(xr + [-1 1]*dx*pad);
ylim(yr + [-1 1]*dy*pad);
zlim(zr + [-1 1]*dz*pad);


%% angular velocity
figure
subplot(3,1,1)
plot(time,desired_angular_velocity(:,1),'-r','LineWidth',2.0); hold on
plot(time,angular_velocity(:,1),'-b','LineWidth',1.0); title('roll\_velocity'); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

subplot(3,1,2)
plot(time,desired_angular_velocity(:,2),'-r','LineWidth',2.0); hold on
plot(time,angular_velocity(:,2),'-b','LineWidth',1.0); title('pitch\_velocity'); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

subplot(3,1,3)
plot(time,desired_angular_velocity(:,3),'-r','LineWidth',2.0); hold on
plot(time,angular_velocity(:,3),'-b','LineWidth',1.0); title('yaw\_velocity'); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

%% Servo_angle
figure
subplot(5,1,1); plot(time,desired_servo_angle(:,1),'-','LineWidth',2.0); hold on; plot(time,servo_angle(:,1),'-','LineWidth',1.0); grid on; title('Servo\_1'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(5,1,2); plot(time,desired_servo_angle(:,2),'-','LineWidth',2.0); hold on; plot(time,servo_angle(:,2),'-','LineWidth',1.0); grid on; title('Servo\_2'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(5,1,3); plot(time,desired_servo_angle(:,3),'-','LineWidth',2.0); hold on; plot(time,servo_angle(:,3),'-','LineWidth',1.0); grid on; title('Servo\_3'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(5,1,4); plot(time,desired_servo_angle(:,4),'-','LineWidth',2.0); hold on; plot(time,servo_angle(:,4),'-','LineWidth',1.0); grid on; title('Servo\_4'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(5,1,5); plot(time,desired_servo_angle(:,5),'-','LineWidth',2.0); hold on; plot(time,servo_angle(:,5),'-','LineWidth',1.0); grid on; title('Servo\_5'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

%% Force
figure
subplot(4,1,1); plot(time,individual_motor_thrust(:,1),'-','LineWidth',2.0); grid on; title('F\_1'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(4,1,2); plot(time,individual_motor_thrust(:,2),'-','LineWidth',2.0); grid on; title('F\_2'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(4,1,3); plot(time,individual_motor_thrust(:,3),'-','LineWidth',2.0); grid on; title('F\_3'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(4,1,4); plot(time,individual_motor_thrust(:,4),'-','LineWidth',2.0); grid on; title('F\_4'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

%% PWM
figure
subplot(4,1,1); plot(time,PWM_cmd(:,1),'-','LineWidth',2.0); grid on; title('pwm1'); ylim([1000 2000]); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(4,1,2); plot(time,PWM_cmd(:,2),'-','LineWidth',2.0); grid on; title('pwm2'); ylim([1000 2000]); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(4,1,3); plot(time,PWM_cmd(:,3),'-','LineWidth',2.0); grid on; title('pwm3'); ylim([1000 2000]); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(4,1,4); plot(time,PWM_cmd(:,4),'-','LineWidth',2.0); grid on; title('pwm4'); ylim([1000 2000]); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

%% Linear Velocity plot
figure
subplot(3,1,1)
plot(time,desired_linear_velocity(:,1),'-r','LineWidth',2.0); hold on
plot(time,linear_velocity(:,1),'-b','LineWidth',2.0);
ylabel('$\bf{\dot{x}}$ \rm\bf{(m/s)}','Interpreter','latex')
legend('$^{G}\dot{x_d}$','$^{G}\dot{x}$','Interpreter','latex','Orientation','horizontal')
title('Velocity'); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

subplot(3,1,2)
plot(time,desired_linear_velocity(:,2),'-r','LineWidth',2.0); hold on
plot(time,linear_velocity(:,2),'-b','LineWidth',2.0);
ylabel('$\bf{\dot{y}}$ \rm\bf{(m/s)}','Interpreter','latex')
legend('$^{G}\dot{y_d}$','$^{G}\dot{y}$','Interpreter','latex','Orientation','horizontal')
grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

subplot(3,1,3)
plot(time,desired_linear_velocity(:,3),'-r','LineWidth',2.0); hold on
plot(time,linear_velocity(:,3),'-b','LineWidth',2.0);
legend('$^{G}\dot{z}$','Interpreter','latex','Orientation','horizontal')
grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

%% desired_torque
figure
subplot(4,1,1); plot(time,desired_torque(:,1),'-','LineWidth',2.0); grid on; title('roll\_torque'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(4,1,2); plot(time,desired_torque(:,2),'-','LineWidth',2.0); grid on; title('pitch\_torque'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(4,1,3); plot(time,desired_torque(:,3),'-','LineWidth',2.0); grid on; title('yaw\_torque'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(4,1,4); plot(time,desired_torque(:,4),'-','LineWidth',2.0); grid on; title('yaw\_torque\_trim'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

%% Desired force
figure
subplot(3,1,1); plot(time,desired_force(:,1),'-b','LineWidth',2.0); grid on; title('Fx\_desired'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(3,1,2); plot(time,desired_force(:,2),'-b','LineWidth',2.0); grid on; title('Fy\_desired'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
subplot(3,1,3); plot(time,desired_force(:,3),'-b','LineWidth',2.0); grid on; title('Fz\_desired'); apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

%% acceleration
windowSize = 100;
figure
subplot(3,1,1)
plot(time,desired_acceleration(:,1),'-r','LineWidth',2.0); hold on
plot(time,movmean(acceleration(:,1), windowSize),'-b','LineWidth',2.0); grid on
title('acceleration\_X')
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

subplot(3,1,2)
plot(time,desired_acceleration(:,2),'-r','LineWidth',2.0); hold on
plot(time,movmean(acceleration(:,2), windowSize),'-b','LineWidth',2.0); grid on
title('acceleration\_Y')
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

subplot(3,1,3)
plot(time,desired_acceleration(:,3),'-r','LineWidth',2.0); hold on
plot(time,acceleration(:,3),'-b','LineWidth',2.0); grid on
title('acceleration\_Z')
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

%% battery_voltage (guarded: 존재할 때만 플롯)
if exist('battery_voltage','var') && ~isempty(battery_voltage)
    figure
    plot(time,battery_voltage(:,1),'-','LineWidth',2.0); grid on
    title('battery\_voltage');
    apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
end

%% DOB disturbance hat
figure('Position',[100 100 600 600], 'Color','w');

ax1 = subplot(3,1,1);
plot(time,torque_dhat(:,1),'-','LineWidth',2.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
ylabel('$\hat{\tau}_x$ [Nm]','Interpreter','latex','FontSize',14);
ylim([-1,0.5])

ax2 = subplot(3,1,2);
plot(time,torque_dhat(:,2),'-','LineWidth',2.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
ylabel('$\hat{\tau}_y$ [Nm]','Interpreter','latex','FontSize',14);
ylim([-0.5,1])

ax3 = subplot(3,1,3);
plot(time,torque_dhat(:,3),'-','LineWidth',2.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('$\hat{\tau}_z$ [Nm]','Interpreter','latex','FontSize',14);
ylim([-0.5,0.5])

% === Y축 한번에 통일 ===

%% Center of Mass Hat
figure('Position',[100 100 600 600], 'Color','w');

ax1 = subplot(3,1,1);
plot(time,past_com(:,1),'-b','LineWidth',1.0); hold on
plot(time,com_update(:,1),'--g','LineWidth',3.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
ylabel('$c_x$ [m]','Interpreter','latex','FontSize',14);

ax2 = subplot(3,1,2);
plot(time,past_com(:,2),'-b','LineWidth',1.0); hold on
plot(time,com_update(:,2),'--g','LineWidth',3.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
ylabel('$c_y$ [m]','Interpreter','latex','FontSize',14);

ax3 = subplot(3,1,3);
plot(time,past_com(:,3),'-b','LineWidth',1.0); hold on
plot(time,com_update(:,3),'--g','LineWidth',3.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('$c_z$ [m]','Interpreter','latex','FontSize',14);

% === Y축 동기화 ===
linkaxes([ax1 ax2],'y');     % x, y는 동기화
ylim(ax1,[-0.1 0.1]);      % x,y 범위
ylim(ax3,[-0.001,0.001]);
%% Servo XY mapping (time-independent)
r = 4;  % radius in cm
theta_offset = pi/4;  % 기준 위치: servo=0일 때 실제 θ=π/4
theta_servo = servo_angle(:,5);         % [rad]
theta_corrected = theta_servo - theta_offset;
X = -r * sin(theta_corrected);  % vertical
Y = -r * cos(theta_corrected);  % horizontal
figure; plot(Y, X, '-'); axis equal; grid on;
xlabel('Y [cm] (Horizontal)'); ylabel('X [cm] (Vertical)');
title('Mapped XY Position with Servo Angle (X=Vertical, Y=Horizontal)');

%% NEW FIGURE
figure
subplot(3,1,1)
plot(time,torque_dhat(:,1),'-b','LineWidth',1.0); hold on
plot(time,-torque_dhat(:,2),'-r','LineWidth',1.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels); ylim([-1.5 2.2]);

subplot(3,1,3)
plot(time,desired_servo_angle(:,5),'-','LineWidth',2.0); hold on
plot(time,servo_angle(:,5),'-','LineWidth',1.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

subplot(3,1,2)
plot(time,com_update(:,1),'--r','LineWidth',3.0); hold on
plot(time,com_update(:,2),'--b','LineWidth',2.0); hold on
plot(time,X/100,'-r','LineWidth',1.0); hold on
plot(time,Y/100,'-b','LineWidth',1.0); grid on
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels); ylim([-0.04 0.04]);

%% Tiled overview
figure
tiledlayout(3,2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile(1)
plot(time, torque_dhat(:,1), '-r', 'LineWidth', 1.8); hold on
plot(time, torque_dhat(:,2), '-b', 'LineWidth', 1.8); hold on
plot(time, 15*com_update(:,2), '--g', 'LineWidth', 2.0); hold on
plot(time, 15*com_update(:,1), '-.k', 'LineWidth', 2.0); grid on
title('Roll/Pitch Torque dhat and 15×CoM\_update');
legend('Roll Torque Dhat', 'Pitch Torque Dhat', '15·CoM\_x', '15·CoM\_y', 'Location','southwest');
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

nexttile(2)
plot(time, desired_servo_angle(:,5), '-', 'LineWidth', 2.0); hold on
plot(time, servo_angle(:,5), '-', 'LineWidth', 1.0); grid on
title('Servo\_5');
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

nexttile([1 2])
plot(time, present_com(:,1), '-r', 'LineWidth', 1.0); hold on
plot(time, past_com(:,1), '-b', 'LineWidth', 1.0); hold on
plot(time, com_update(:,1), '--g', 'LineWidth', 2.0); grid on
ylim([-0.03 0.04]); title('CoM\_hat\_x');
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

nexttile([1 2])
plot(time, present_com(:,2), '-r', 'LineWidth', 1.0); hold on
plot(time, past_com(:,2), '-b', 'LineWidth', 1.0); hold on
plot(time, com_update(:,2), '--g', 'LineWidth', 2.0); grid on
title('CoM\_hat\_y');
apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels);

%% ===== Helpers =====
function apply_xlim_xticks(x_start, x_end, xtick_vals, xtick_labels)
    xlim([x_start x_end]);
    xticks(xtick_vals);
    xticklabels(xtick_labels);
end
