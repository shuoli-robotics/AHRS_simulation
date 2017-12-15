function [] = generate_IMU_measurement()
% According to ground truth maneuver, this fumction
% is used to generated noisy and biased IMU measurement

global GT IMU

acc_bias = [-0.0 0.0 -0.0];
gyro_bias = [0.2/180*pi -0.2/180*pi 0.3/180*pi];

acc_sigma = 5;
gyro_sigma = 0.2/180*pi;
att_sigma = 0.3/180*pi;
OT_sigma = 0.3/180*pi;

IMU.ACC = zeros(size(GT.ACC));
IMU.PQR = zeros(size(GT.PQR));
IMU.OT_HEADING = zeros(length(GT.TIME),1);
IMU.PHI = zeros(length(GT.TIME),1);
IMU.THETA = zeros(length(GT.TIME),1);
IMU.ACC_SIGMA = acc_sigma;
IMU.GYRO_SIGMA = gyro_sigma;
IMU.OT_SIGMA = OT_sigma;
IMU.ATT_SIGMA = att_sigma;
IMU.ACC_BIAS = acc_bias;
IMU.GYRO_BIAS = gyro_bias;


for i = 1:length(GT.TIME)
    IMU.ACC(i,:) = GT.ACC(i,:) + acc_bias+[normrnd(0,acc_sigma) normrnd(0,acc_sigma) normrnd(0,acc_sigma)];
    IMU.PQR(i,:) = GT.PQR(i,:) + gyro_bias+[normrnd(0,gyro_sigma) normrnd(0,gyro_sigma) normrnd(0,gyro_sigma)];
    IMU.OT_HEADING(i) = GT.PSI(i,:)+normrnd(0,OT_sigma);
    IMU.PHI(i) = GT.PHI(i)+ normrnd(0,att_sigma);
    IMU.THETA(i) = GT.THETA(i)+ normrnd(0,att_sigma);
end

for i = 1:length(GT.TIME)
    euler_from_accel(i,1) = atan2(-IMU.ACC(i,2),-IMU.ACC(i,3));
    euler_from_accel(i,2) = atan2(IMU.ACC(i,1),sqrt(IMU.ACC(i,2)^2+IMU.ACC(i,3)^2));
end
accel_error = euler_from_accel-[GT.PHI GT.THETA];
accel_mean = [mean(accel_error(:,1)) mean(accel_error(:,2))];
accel_var = [std(accel_error(:,1)) std(accel_error(:,2))];

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

% figure(2)
% subplot(3,1,1)
% plot(GT.TIME,GT.PHI(:,1));
% hold on
% plot(GT.TIME,IMU.PHI(:,1));
% 
% subplot(3,1,2)
% plot(GT.TIME,GT.THETA);
% hold on
% plot(GT.TIME,IMU.THETA);
% 
% subplot(3,1,3)
% plot(GT.TIME,GT.PSI);
% hold on
% plot(GT.TIME,IMU.OT_HEADING);

% figure(1)
% plot(GT.TIME,IMU.OT_HEADING/pi*180);
%--------------------------------------------------------------------------%
end