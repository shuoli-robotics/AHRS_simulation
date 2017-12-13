function [] = EKF_6_STATES()
global GT IMU EKF
[Fx_syms,Hx_syms,G_syms,model,h,states] = jacobian_matrix_syms();
EKF_states = zeros(length(GT.TIME),6);
P_trace = zeros(length(GT.TIME),6);

Q = [0 0 0]';
R = diag([20 20])';

for i = 1:length(GT.TIME)
    if i == 1
        EKF_states(i,:) = [GT.PHI(i) GT.THETA(i) GT.PSI(i) 0 0 0];
        last_updated_states_EKF =  EKF_states(i,:);
        last_updated_inputs = [IMU.PQR(i,:)];
        P_k_k = 10*eye(6);
        P_trace(i) = trace(P_k_k);
        continue;
    end
  
    inputs = IMU.PQR(i,:);
    EKF_states(i,:) = EKF_states(i-1,:) + prediction_model_kalman_filter(EKF_states(i-1,:),inputs)*GT.STEP;
    [F,H_k,G] = jacobian_matrix(last_updated_states_EKF,last_updated_inputs,EKF_states(i,:),...
        Fx_syms,Hx_syms,G_syms);
    [PHI_k_k_1, Gamma] = c2d(F, G, GT.STEP);
    P_k_1_k_1 = P_k_k;
    P_k_k_1 = PHI_k_k_1*P_k_1_k_1*PHI_k_k_1'+Gamma*diag(Q)*Gamma';
    Z_k_k_1 = EKF_states(i,1:2);
    Z_k = [atan2(-GT.ACC(i,2),-GT.ACC(i,3)), atan2(GT.ACC(i,1),sqrt(GT.ACC(i,2)^2+GT.ACC(i,3)^2))]';
    K_k = P_k_k_1*H_k'/ (H_k*P_k_k_1*H_k'+R);
    delta_x_k_k = K_k*(Z_k-Z_k_k_1');
    EKF_states(i,:) = EKF_states(i,:) + delta_x_k_k';
    
    EKF.PHI(i) = EKF_states(i,1);
    EKF.THETA(i) = EKF_states(i,2);
    EKF.PSI(i) = EKF_states(i,3);
    EKF.B_P(i) = EKF_states(i,4);
    EKF.B_Q(i) = EKF_states(i,5);
    EKF.B_R(i) = EKF_states(i,6);
end


figure(1)
subplot(3,1,1)
plot(GT.TIME,GT.PHI/pi*180);
hold on
plot(GT.TIME,EKF.PHI/pi*180);


subplot(3,1,2)
plot(GT.TIME,GT.THETA/pi*180);
hold on
plot(GT.TIME,EKF.THETA/pi*180);

subplot(3,1,3)
plot(GT.TIME,GT.PSI/pi*180);
hold on
plot(GT.TIME,EKF.PSI/pi*180);

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
