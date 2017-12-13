function [F,H,G] = jacobian_matrix(estimated_states,inputs,one_step_prediction,Fx_syms,Hx_syms,G_syms)
% syms x y z v_x v_y v_z real;
% syms phi theta psi
% syms b_p b_q b_r
% syms b_a_x b_a_y b_a_z
% syms a_x_m a_y_m a_z_m
% syms p_m q_m r_m
% syms g 

phi = estimated_states(1);
theta = estimated_states(2);
psi = estimated_states(3);
b_p = estimated_states(4);
b_q = estimated_states(5);
b_r = estimated_states(6);

p_m = inputs(1);
q_m = inputs(2);
r_m = inputs(3);


% F = subs(Fx_syms,{x y z v_x v_y v_z phi theta psi b_p b_q b_r b_a_x b_a_y b_a_z ...
%     p_m q_m r_m a_x_m a_y_m a_z_m g},...
%     {[estimated_states inputs 9.8]});
% F = double(F);
% 
% H = double(subs(Hx_syms,{x y z v_x v_y v_z phi theta psi b_p b_q b_r b_a_x b_a_y b_a_z},...
%     {one_step_prediction}));
% 
% 
% G = subs(G_syms,{phi theta psi},{estimated_states(7:9)});
% G = double(G);

F = [     sin(phi)*tan(theta)*(b_r - r_m) - cos(phi)*tan(theta)*(b_q - q_m),               - cos(phi)*(b_r - r_m)*(tan(theta)^2 + 1) - sin(phi)*(b_q - q_m)*(tan(theta)^2 + 1), 0, -1, -sin(phi)*tan(theta), -cos(phi)*tan(theta); ...
    cos(phi)*(b_r - r_m) + sin(phi)*(b_q - q_m),                                                                                                 0, 0,  0,            -cos(phi),             sin(phi); ...
    (sin(phi)*(b_r - r_m))/cos(theta) - (cos(phi)*(b_q - q_m))/cos(theta), - (cos(phi)*sin(theta)*(b_r - r_m))/cos(theta)^2 - (sin(phi)*sin(theta)*(b_q - q_m))/cos(theta)^2, 0,  0, -sin(phi)/cos(theta), -cos(phi)/cos(theta); ...
    0,                                                                                                 0, 0,  0,                    0,                    0; ...
    0,                                                                                                 0, 0,  0,                    0,                    0; ...
    0,                                                                                                 0, 0,  0,                    0,                    0];

H = [ 1, 0, 0, 0, 0, 0; ...
    0, 1, 0, 0, 0, 0];

G = [ 1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
    0,            cos(phi),           -sin(phi);...
    0, sin(phi)/cos(theta), cos(phi)/cos(theta);...
    0,                   0,                   0;...
    0,                   0,                   0;...
    0,                   0,                   0];

  