function [] =complementary_filter()
% This function is used to simulate on-board complementary 
% filter. It is actually a low pass filter

global GT IMU COMP

COMP.PHI = zeros(length(GT.TIME));
COMP.THETA = zeros(length(GT.TIME));
COMP.PSI = zeros(length(GT.TIME));

K_P = 1;
K_I = 0.1;
sum_error = 0;

for i = 1:length(GT.TIME)
    if i == 1
        COMP.PHI(i) = GT.PHI(i);
        COMP.THETA(i) = GT.THETA(i);
        COMP.PSI(i) = GT.PSI(i);
        continue;
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
end

end