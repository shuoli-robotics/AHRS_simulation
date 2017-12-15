function [] =complementary_filter()
% This function is used to simulate on-board complementary 
% filter. It is actually a low pass filter

global GT IMU COMP

COMP.PHI = zeros(length(GT.TIME),1);
COMP.THETA = zeros(length(GT.TIME),1);
COMP.PSI = zeros(length(GT.TIME),1);

K_P = 0.01;
K_I = 0.002;
sum_error = 0;

for i = 1:length(GT.TIME)
    if i == 1
        COMP.PHI(i) = GT.PHI(i);
        COMP.THETA(i) = GT.THETA(i);
        COMP.PSI(i) = GT.PSI(i);
    end
    R_E_B = [cos(GT.THETA(i))*cos(GT.PSI(i)) cos(GT.THETA(i))*sin(GT.PSI(i)) -sin(GT.THETA(i));...
        sin(GT.PHI(i))*sin(GT.THETA(i))*cos(GT.PSI(i))-cos(GT.PHI(i))*sin(GT.PSI(i))...
        sin(GT.PHI(i))*sin(GT.THETA(i))*sin(GT.PSI(i))+cos(GT.PHI(i))*cos(GT.PSI(i)) sin(GT.PHI(i))*cos(GT.THETA(i));...
        cos(GT.PHI(i))*sin(GT.THETA(i))*cos(GT.PSI(i))+sin(GT.PHI(i))*sin(GT.PSI(i))...
        cos(GT.PHI(i))*sin(GT.THETA(i))*sin(GT.PSI(i))-sin(GT.PHI(i))*cos(GT.PSI(i)) cos(GT.PHI(i))*cos(GT.THETA(i))];
    g_b = R_E_B*[0 0 -9.8]';
    g_b = g_b/norm(g_b);
    acc_m = IMU.ACC(i,:)'/norm(IMU.ACC(i,:));
    error = cross(acc_m,g_b);
    sum_error = sum_error + error*GT.STEP;
    PQR  = K_P * error + K_I * sum_error + IMU.PQR(i,:)';
    R_q_phi = [1 tan(COMP.THETA(i))*sin(COMP.PHI(i)) tan(COMP.THETA(i))*cos(COMP.PHI(i));...
        0 cos(COMP.PHI(i)) -sin(COMP.PHI(i));...
        0 sin(COMP.PHI(i))/cos(COMP.THETA(i)) cos(COMP.PHI(i))/cos(COMP.THETA(i))];
    d_att = R_q_phi * PQR;
    att = [COMP.PHI(i) COMP.THETA(i) COMP.PSI(i)] + d_att'*GT.STEP;
    if i == length(GT.TIME)
       break; 
    end
    COMP.PHI(i+1) = att(1);
    COMP.THETA(i+1) = att(2);
    COMP.PSI(i+1) = att(3);
end

%-----------------------------test generated data  -----------------------------%
%--------------------------------------------------------------------------%
figure(1)
subplot(3,1,1)
plot(GT.TIME,GT.PHI/pi*180);
hold on
plot(GT.TIME,COMP.PHI/pi*180);

subplot(3,1,2)
plot(GT.TIME,GT.THETA/pi*180);
hold on
plot(GT.TIME,COMP.THETA/pi*180);

subplot(3,1,3)
plot(GT.TIME,GT.PSI/pi*180);
hold on
plot(GT.TIME,COMP.PSI/pi*180);

figure(2)
subplot(3,1,1)
plot(GT.TIME,GT.PQR(:,1));
hold on
plot(GT.TIME,IMU.PQR(:,1));

subplot(3,1,2)
plot(GT.TIME,GT.PQR(:,2));
hold on
plot(GT.TIME,IMU.PQR(:,2));

subplot(3,1,3)
plot(GT.TIME,GT.PQR(:,3));
hold on
plot(GT.TIME,IMU.PQR(:,3));
%--------------------------------------------------------------------------%

end