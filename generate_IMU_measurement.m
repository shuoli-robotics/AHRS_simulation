function [] = generate_IMU_measurement()
% According to ground truth maneuver, this fumction
% is used to generated noisy and biased IMU measurement

global GT IMU

acc_bias = [0 0 0];
gyro_bias = [0 0 0];

acc_sigma = 20;
gyro_sigma = 0;

IMU.ACC = zeros(size(GT.ACC));
IMU.PQR = zeros(size(GT.PQR));

for i = 1:length(GT.TIME)
    IMU.ACC(i,:) = GT.ACC(i,:) + acc_bias+[normrnd(0,acc_sigma) normrnd(0,acc_sigma) normrnd(0,acc_sigma)];
    IMU.PQR(i,:) = GT.PQR(i,:) + gyro_bias+[normrnd(0,gyro_sigma) normrnd(0,gyro_sigma) normrnd(0,gyro_sigma)];
end

%-----------------------------test generated data  -----------------------------%
%--------------------------------------------------------------------------%
% figure(1)
% subplot(3,1,1)
% plot(GT.TIME,GT.ACC(:,1));
% hold on
% plot(GT.TIME,IMU.ACC(:,1));
% 
% subplot(3,1,2)
% plot(GT.TIME,GT.ACC(:,2));
% hold on
% plot(GT.TIME,IMU.ACC(:,2));
% 
% subplot(3,1,3)
% plot(GT.TIME,GT.ACC(:,3));
% hold on
% plot(GT.TIME,IMU.ACC(:,3));
% 
% figure(2)
% subplot(3,1,1)
% plot(GT.TIME,GT.PQR(:,1));
% hold on
% plot(GT.TIME,IMU.PQR(:,1));
% 
% subplot(3,1,2)
% plot(GT.TIME,GT.PQR(:,2));
% hold on
% plot(GT.TIME,IMU.PQR(:,2));
% 
% subplot(3,1,3)
% plot(GT.TIME,GT.PQR(:,3));
% hold on
% plot(GT.TIME,IMU.PQR(:,3));
%--------------------------------------------------------------------------%
end