function [] = generate_ground_truth_maneuver(mode,time)
% This function is used to generate ground truth 
% phi theta psi and also corresponding p q r
% At last, ground truth specific acceleration is also 
% generated.
% Mode indicates id of ground truth mode
% time is simulation time

global GT

step = 0.01;
phi0 = -pi/6;
theta0 = pi/6;
psi0 = pi/2;
omega_phi = pi/4;
omega_theta = pi/6;
omega_psi = pi/8;

GT.STEP = step;

GT.TIME = zeros(time/step,1);
GT.PHI = zeros(time/step,1);
GT.THETA = zeros(time/step,1);
GT.PSI = zeros(time/step,1);
GT.PQR = zeros(time/step,3);
GT.ACC = zeros(time/step,3);

for i = 1:time/step
    t = i*step;
    GT.TIME(i) = t;
    switch mode
        case 1
            GT.PHI(i) = phi0*sin(omega_phi*t);
            GT.THETA(i) = theta0*sin(omega_theta*t);
            GT.PSI(i) = psi0*sin(omega_psi*t);
            d_phi = omega_phi*phi0*cos(omega_phi*t);
            d_theta = omega_theta*theta0*cos(omega_theta*t);
            d_psi = omega_psi*psi0*cos(omega_psi*t);
            temp = [1 0 -sin(GT.THETA(i));...
                0 cos(GT.PHI(i)) cos(GT.THETA(i))*sin(GT.PHI(i));...
                0 -sin(GT.PHI(i)) cos(GT.THETA(i))*cos(GT.PHI(i))]*[d_phi d_theta d_psi]';
            GT.PQR(i,1) = temp(1);
            GT.PQR(i,2) = temp(2);
            GT.PQR(i,3) = temp(3);
            ACCEL =  [cos(GT.THETA(i))*cos(GT.PSI(i)) cos(GT.THETA(i))*sin(GT.PSI(i)) -sin(GT.THETA(i));...
                sin(GT.PHI(i))*sin(GT.THETA(i))*cos(GT.PSI(i))-cos(GT.PHI(i))*sin(GT.PSI(i))...
                sin(GT.PHI(i))*sin(GT.THETA(i))*sin(GT.PSI(i))+cos(GT.PHI(i))*cos(GT.PSI(i)) sin(GT.PHI(i))*cos(GT.THETA(i));...
                cos(GT.PHI(i))*sin(GT.THETA(i))*cos(GT.PSI(i))+sin(GT.PHI(i))*sin(GT.PSI(i))...
                cos(GT.PHI(i))*sin(GT.THETA(i))*sin(GT.PSI(i))-sin(GT.PHI(i))*cos(GT.PSI(i)) cos(GT.PHI(i))*cos(GT.THETA(i))]*[0 0 -9.8]';
            GT.ACC(i,:)  = ACCEL';
            
        case 2
            GT.PHI(i) = 0;
            GT.THETA(i) = 0;
            GT.PSI(i) = 0;
            GT.PQR(i,1) = 0;
            GT.PQR(i,2) = 0;
            GT.PQR(i,3) = 0;
            ACCEL =  [cos(GT.THETA(i))*cos(GT.PSI(i)) cos(GT.THETA(i))*sin(GT.PSI(i)) -sin(GT.THETA(i));...
                sin(GT.PHI(i))*sin(GT.THETA(i))*cos(GT.PSI(i))-cos(GT.PHI(i))*sin(GT.PSI(i))...
                sin(GT.PHI(i))*sin(GT.THETA(i))*sin(GT.PSI(i))+cos(GT.PHI(i))*cos(GT.PSI(i)) sin(GT.PHI(i))*cos(GT.THETA(i));...
                cos(GT.PHI(i))*sin(GT.THETA(i))*cos(GT.PSI(i))+sin(GT.PHI(i))*sin(GT.PSI(i))...
                cos(GT.PHI(i))*sin(GT.THETA(i))*sin(GT.PSI(i))-sin(GT.PHI(i))*cos(GT.PSI(i)) cos(GT.PHI(i))*cos(GT.THETA(i))]*[0 0 -9.8]';
            GT.ACC(i,:)  = ACCEL';
    end
end
% %-----------------------------test generated data  -----------------------------%
% %--------------------------------------------------------------------------%
% for i = 1:time/step
%     euler_from_accel(i,1) = atan2(-GT.ACC(i,2),-GT.ACC(i,3));
%     euler_from_accel(i,2) = atan2(GT.ACC(i,1),sqrt(GT.ACC(i,2)^2+GT.ACC(i,3)^2));
% end
% 
% figure(1)
% subplot(3,1,1)
% plot(GT.TIME,GT.PHI);
% hold on
% plot(GT.TIME,euler_from_accel(:,1));
% 
% subplot(3,1,2)
% plot(GT.TIME,GT.THETA);
% hold on
% plot(GT.TIME,euler_from_accel(:,2));
% 
% subplot(3,1,3)
% plot(GT.TIME,GT.PSI);

% %--------------------------------------------------------------------------%
end