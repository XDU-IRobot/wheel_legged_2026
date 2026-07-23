% =========================================================================
% 终极融合版：微分方程基石 + 拉格朗日非线性梯度补丁 + SSTO(稳态流形映射)
% =========================================================================
tic
clearvars
clc
fprintf('Step 0: 初始化符号变量...\n');
syms R_w R_l l_l l_r l_wl l_wr l_bl l_br l_c real
syms m_w m_l m_b I_w I_ll I_lr I_b I_z g real
syms theta_wl theta_wr dtheta_wl dtheta_wr ddtheta_wl ddtheta_wr ddtheta_ll ddtheta_lr ddtheta_b
syms theta_ll theta_lr dtheta_ll dtheta_lr
syms theta_b dtheta_b
syms T_wl T_wr T_bl T_br real
q  = [theta_wl; theta_wr; theta_ll; theta_lr; theta_b];
dq = [dtheta_wl; dtheta_wr; dtheta_ll; dtheta_lr; dtheta_b];

% =========================================================================
% [PART 1]: 你的完美微分方程 (提取基准 A, B, M, G, B_tau)
% =========================================================================
fprintf('Step 1A: 正在使用原版微分方程提取基准矩阵...\n');
eq1_lhs = (I_w*l_l/R_w+m_w*R_w*l_l+m_l*R_w*l_bl)*ddtheta_wl+(m_l*l_wl*l_bl-I_ll)*ddtheta_ll+(m_l*l_wl+m_b*l_l/2)*g*theta_ll+T_bl-T_wl*(1+l_l/R_w);
eq2_lhs = (I_w*l_r/R_w+m_w*R_w*l_r+m_l*R_w*l_br)*ddtheta_wr+(m_l*l_wr*l_br-I_lr)*ddtheta_lr+(m_l*l_wr+m_b*l_r/2)*g*theta_lr+T_br-T_wr*(1+l_r/R_w);
eq3_lhs = -(m_w*R_w*R_w+I_w+m_l*R_w*R_w+m_b*R_w*R_w/2)*ddtheta_wl-(m_w*R_w*R_w+I_w+m_l*R_w*R_w+m_b*R_w*R_w/2)*ddtheta_wr-(m_l*R_w*l_wl+m_b*R_w*l_l/2)*ddtheta_ll-(m_l*R_w*l_wr+m_b*R_w*l_r/2)*ddtheta_lr+T_wl+T_wr;
eq4_lhs = (m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wl+(m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wr+m_l*l_wl*l_c*ddtheta_ll+m_l*l_wr*l_c*ddtheta_lr-I_b*ddtheta_b+m_b*g*l_c*theta_b-(T_wl+T_wr)*l_c/R_w-(T_bl+T_br);
eq5_lhs = ((I_z*R_w)/(2*R_l)+I_w*R_l/R_w)*ddtheta_wl-((I_z*R_w)/(2*R_l)+I_w*R_l/R_w)*ddtheta_wr+(I_z*l_l)/(2*R_l)*ddtheta_ll-(I_z*l_r)/(2*R_l)*ddtheta_lr-T_wl*R_l/R_w+T_wr*R_l/R_w;
eqn1 = eq1_lhs == 0; eqn2 = eq2_lhs == 0; eqn3 = eq3_lhs == 0; eqn4 = eq4_lhs == 0; eqn5 = eq5_lhs == 0;
[ddtheta_wl_sol,ddtheta_wr_sol,ddtheta_ll_sol,ddtheta_lr_sol,ddtheta_b_sol] = ...
    solve(eqn1,eqn2,eqn3,eqn4,eqn5,ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b);

J_A = jacobian([ddtheta_wl_sol,ddtheta_wr_sol,ddtheta_ll_sol,ddtheta_lr_sol,ddtheta_b_sol],[theta_ll,theta_lr,theta_b]);
J_B = jacobian([ddtheta_wl_sol,ddtheta_wr_sol,ddtheta_ll_sol,ddtheta_lr_sol,ddtheta_b_sol],[T_wl,T_wr,T_bl,T_br]);

A_base = sym('A',[10 10]); B_base = sym('B',[10 4]);
for p = 5:2:9
    A_index = (p - 3)/2;
    A_base(2,p) = R_w*(J_A(1,A_index) + J_A(2,A_index))/2;
    A_base(4,p) = (R_w*(- J_A(1,A_index) + J_A(2,A_index)))/(2*R_l) - (l_l*J_A(3,A_index))/(2*R_l) + (l_r*J_A(4,A_index))/(2*R_l);
    for qq = 6:2:10, A_base(qq,p) = J_A(qq/2,A_index); end
end
for r = 1:10
    if rem(r,2) == 0, A_base(r,1)=0; A_base(r,2)=0; A_base(r,3)=0; A_base(r,4)=0; A_base(r,6)=0; A_base(r,8)=0; A_base(r,10)=0;
    else, A_base(r,:) = zeros(1,10); A_base(r,r+1) = 1; end
end
for h = 1:4
    B_base(2,h) = R_w*(J_B(1,h) + J_B(2,h))/2;
    B_base(4,h) = (R_w*(- J_B(1,h) + J_B(2,h)))/(2*R_l) - (l_l*J_B(3,h))/(2*R_l) + (l_r*J_B(4,h))/(2*R_l);
    for f = 6:2:10, B_base(f,h) = J_B(f/2,h); end
end
for e = 1:2:9, B_base(e,:) = zeros(1,4); end

% MDO 的原生矩阵 (完美解耦版)
Q_var   = [theta_wl; theta_wr; theta_ll; theta_lr; theta_b];
ddQ_var = [ddtheta_wl; ddtheta_wr; ddtheta_ll; ddtheta_lr; ddtheta_b];
U_var   = [T_wl; T_wr; T_bl; T_br];
Eq_vec  = [eq1_lhs; eq2_lhs; eq3_lhs; eq4_lhs; eq5_lhs];
M_sym     = jacobian(Eq_vec, ddQ_var);
G_sym     = jacobian(Eq_vec, Q_var);
B_tau_sym = -jacobian(Eq_vec, U_var);

% =========================================================================
% [PART 1.5]: 预定义 SSTO (稳态目标优化器) 的绝对硬约束矩阵
% =========================================================================
% SSTO 要求解: [I - Ad, -Bd] * [x_ss; u_ss] = Bd * D_steady
% 并且强制服从以下死规矩: 不溜车(速度为0,位置为0)，机身绝不低头(Pitch=0)
C_hard = zeros(6, 14); 
C_hard(1, 1) = 1;  % x[0] (左轮位移) = 0
C_hard(2, 3) = 1;  % x[2] (右轮位移) = 0
C_hard(3, 9) = 1;  % x[8] (Pitch角度) = 0
C_hard(4, 2) = 1;  % x[1] (左轮速) = 0
C_hard(5, 4) = 1;  % x[3] (右轮速) = 0
C_hard(6, 10) = 1; % x[9] (Pitch角速度) = 0

% =========================================================================
% [PART 2]: 拉格朗日势能法 (提取精准的非线性梯度 dAd)
% =========================================================================
fprintf('Step 1B: 正在使用拉格朗日法提取非线性势能梯度...\n');
m_half = m_b / 2; I_half = I_b / 2;
y_ll = R_w  + l_wl * cos(theta_ll);
y_lr = R_w  + l_wr * cos(theta_lr);
y_bl = R_w  + l_l * cos(theta_ll) + l_c * cos(theta_b);
y_br = R_w  + l_r * cos(theta_lr) + l_c * cos(theta_b);
V_total = m_l*g*y_ll + m_l*g*y_lr + m_half*g*y_bl + m_half*g*y_br;
K_sym_nonlin = hessian(V_total, q);
K_flat = reshape(K_sym_nonlin, 1, 25);
syms C_ll C_lr C_b real
K_flat_sub = subs(K_flat, [cos(theta_ll), cos(theta_lr), cos(theta_b)], [C_ll, C_lr, C_b]);
dK_ll_sym = reshape(subs(jacobian(K_flat_sub, C_ll), [theta_ll, theta_lr, theta_b], [0, 0, 0]), 5, 5);
dK_lr_sym = reshape(subs(jacobian(K_flat_sub, C_lr), [theta_ll, theta_lr, theta_b], [0, 0, 0]), 5, 5);
dK_b_sym  = reshape(subs(jacobian(K_flat_sub, C_b),  [theta_ll, theta_lr, theta_b], [0, 0, 0]), 5, 5);

M_sym_eval = subs(M_sym, [theta_ll, theta_lr, theta_b], [0,0,0]);
dA_raw_ll = [zeros(5,5), zeros(5,5); -M_sym_eval \ dK_ll_sym, zeros(5,5)];
dA_raw_lr = [zeros(5,5), zeros(5,5); -M_sym_eval \ dK_lr_sym, zeros(5,5)];
dA_raw_b  = [zeros(5,5), zeros(5,5); -M_sym_eval \ dK_b_sym,  zeros(5,5)];

% 极性映射
T_trans = sym(zeros(10,10));
T_trans(1,1)=0.5*R_w; T_trans(1,2)=0.5*R_w; 
T_trans(2,6)=0.5*R_w; T_trans(2,7)=0.5*R_w;
T_trans(3,1)=-R_w/(2*R_l); T_trans(3,2)=R_w/(2*R_l); T_trans(3,3)=-l_l/(2*R_l); T_trans(3,4)=l_r/(2*R_l);
T_trans(4,6)=-R_w/(2*R_l); T_trans(4,7)=R_w/(2*R_l); T_trans(4,8)=-l_l/(2*R_l); T_trans(4,9)=l_r/(2*R_l);
T_trans(5,3)=1; T_trans(6,8)=1; T_trans(7,4)=1; T_trans(8,9)=1; T_trans(9,5)=1; T_trans(10,10)=1;
dA_dtheta_ll_sym = T_trans * dA_raw_ll / T_trans;
dA_dtheta_lr_sym = T_trans * dA_raw_lr / T_trans;
dA_dtheta_b_sym  = T_trans * dA_raw_b  / T_trans;

fprintf('Step 1.8: 编译全部系统矩阵数值函数...\n');
vars_list = {R_w, R_l, l_l, l_r, l_wl, l_wr, l_bl, l_br, l_c, m_w, m_l, m_b, I_w, I_ll, I_lr, I_b, I_z, g};
A_func     = matlabFunction(A_base,    'Vars', vars_list);
B_func     = matlabFunction(B_base,    'Vars', vars_list);
M_func     = matlabFunction(M_sym,     'Vars', vars_list);
G_func     = matlabFunction(G_sym,     'Vars', vars_list);
B_tau_func = matlabFunction(B_tau_sym, 'Vars', vars_list);
dA_ll_func = matlabFunction(dA_dtheta_ll_sym, 'Vars', vars_list);
dA_lr_func = matlabFunction(dA_dtheta_lr_sym, 'Vars', vars_list);
dA_b_func  = matlabFunction(dA_dtheta_b_sym,  'Vars', vars_list);

%%%%%%%%%%%%%%%%%%%%%Step 2：输入参数%%%%%%%%%%%%%%%%%%%%%
g_ac = 9.81;
R_w_ac = 0.0645; R_l_ac = 0.203; 
l_c_ac = 0.037; 
m_w_ac = 0.245; m_l_ac = 2.019; m_b_ac = 24.242;
I_w_ac = 0.000723; I_b_ac = 0.50; I_z_ac = 0.496;
Leg_data_l = [0.17, 0.1356, 0.08948, 0.03141; 0.18, 0.1388, 0.0928, 0.032051; 0.19, 0.1421, 0.09623, 0.032713;
              0.20, 0.1456, 0.09976, 0.033398; 0.21, 0.1491, 0.1033, 0.034110; 0.22, 0.1527, 0.1070, 0.034848;
              0.23, 0.1564, 0.1108, 0.035615; 0.24, 0.1603, 0.1146, 0.036411; 0.25, 0.1641, 0.1184, 0.037237;
              0.26, 0.1681, 0.1257, 0.038093; 0.27, 0.1721, 0.1263, 0.038980; 0.28, 0.17614, 0.13027, 0.039898;
              0.29, 0.1802, 0.13429, 0.040848; 0.30, 0.1843, 0.1383, 0.041829; 0.31, 0.1801, 0.1424, 0.042842;
              0.326, 0.1954, 0.1492, 0.044584];
Leg_data_r = Leg_data_l;

lqr_Q_hard = diag([ 20, 50,  8,  4, 200,  0.5,  200,  0.5, 800,  0.1 ]);
lqr_R_hard = diag([2.2, 2.2, 0.2, 0.2]);

% lqr_Q_hard = diag([ 3, 5,  5,  30, 100,  2,  100,  2, 200,  1]);
% lqr_R_hard = diag([0.8, 0.8, 0.15, 0.15]);

lqr_Q_soft = diag([ 2, 2,  5,  12, 230,  3, 230,  3, 700,  2]);
lqr_R_soft = diag([0.9,0.9, 0.2, 0.2]);
Ts = 0.002;

%%%%%%%%%%%%%%%%%%%%%%%%%%Step 3：并行计算%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Step 3: 并行计算数据点 (包含 SSTO 解算)...\n');
sample_size  = size(Leg_data_l,1)^2;
length_data  = size(Leg_data_l,1);
params_list  = zeros(sample_size, 8);
k_idx = 1;
for i = 1:length_data
    for j = 1:length_data
        params_list(k_idx, :) = [Leg_data_l(i,:), Leg_data_r(j,:)];
        k_idx = k_idx + 1;
    end
end

% 结果矩阵扩容：增加 14x4 = 56 个元素用于存放 SSTO 映射矩阵 (M_x_ss 和 M_u_ss)
results = zeros(sample_size, 1464); 

if isempty(gcp('nocreate')), parpool; end
parfor idx = 1:sample_size
    p = params_list(idx, :);
    A_val = A_func(R_w_ac, R_l_ac, p(1), p(5), p(2), p(6), p(3), p(7), l_c_ac, ...
                   m_w_ac, m_l_ac, m_b_ac, I_w_ac, p(4), p(8), I_b_ac, I_z_ac, g_ac);
    B_val = B_func(R_w_ac, R_l_ac, p(1), p(5), p(2), p(6), p(3), p(7), l_c_ac, ...
                   m_w_ac, m_l_ac, m_b_ac, I_w_ac, p(4), p(8), I_b_ac, I_z_ac, g_ac);
    
    [P_hard_val, K_hard_val, ~] = icare(A_val, B_val, lqr_Q_hard, lqr_R_hard, [], [], []);
    [P_soft_val, K_soft_val, ~] = icare(A_val, B_val, lqr_Q_soft, lqr_R_soft, [], [], []);
    
    Aa    = [A_val, B_val; zeros(4,10), zeros(4,4)];
    H     = [Aa, [B_val; zeros(4,4)]; zeros(4,14), zeros(4,4)] * Ts;
    M_exp = expm(H);
    Ad_val = M_exp(1:14, 1:14);
    Bd_val = M_exp(1:14, 15:18);
    
% === 重力修正项 (pitch相关) - ZOH离散化 ===
A_val_g0    = A_func(R_w_ac, R_l_ac, p(1), p(5), p(2), p(6), p(3), p(7), l_c_ac, ...
                     m_w_ac, m_l_ac, m_b_ac, I_w_ac, p(4), p(8), I_b_ac, I_z_ac, 0.0);
A_grav_corr = A_val - A_val_g0;

% baseline 不含重力的离散化
Aa_g0    = [A_val_g0, B_val; zeros(4,10), zeros(4,4)];
H_g0     = [Aa_g0, [B_val; zeros(4,4)]; zeros(4,14), zeros(4,4)] * Ts;
M_exp_g0 = expm(H_g0);
Ad_g0    = M_exp_g0(1:14, 1:14);

% Ad_pitch_val 现在表示: (含重力的 Ad) - (不含重力的 Ad)
% 注意: Ad_val 的第10列(pitch列)已经包含了重力效应, 这里提取的是"重力贡献的离散增量"
Ad_pitch_full = Ad_val - Ad_g0;
Ad_pitch_val  = [Ad_pitch_full(1:10, 1:10), zeros(10,4); zeros(4,14)];

% === 三个 dA 修正项 - ZOH离散化 ===
dAc_ll = dA_ll_func(R_w_ac, R_l_ac, p(1), p(5), p(2), p(6), p(3), p(7), l_c_ac, ...
                    m_w_ac, m_l_ac, m_b_ac, I_w_ac, p(4), p(8), I_b_ac, I_z_ac, g_ac);
dAc_lr = dA_lr_func(R_w_ac, R_l_ac, p(1), p(5), p(2), p(6), p(3), p(7), l_c_ac, ...
                    m_w_ac, m_l_ac, m_b_ac, I_w_ac, p(4), p(8), I_b_ac, I_z_ac, g_ac);
dAc_b  = dA_b_func (R_w_ac, R_l_ac, p(1), p(5), p(2), p(6), p(3), p(7), l_c_ac, ...
                    m_w_ac, m_l_ac, m_b_ac, I_w_ac, p(4), p(8), I_b_ac, I_z_ac, g_ac);

% 对每个 dAc 做 ZOH 离散化: 计算 (A_val + dAc) 的 ZOH 减去 A_val 的 ZOH
% 这样得到的就是"添加 dAc 这个连续梯度后，离散矩阵的真实增量"
Aa_ll    = [A_val + dAc_ll, B_val; zeros(4,10), zeros(4,4)];
H_ll     = [Aa_ll, [B_val; zeros(4,4)]; zeros(4,14), zeros(4,4)] * Ts;
M_exp_ll = expm(H_ll);
dAd_ll_full = M_exp_ll(1:14, 1:14) - Ad_val;
dAd_ll_val  = [dAd_ll_full(1:10, 1:10), zeros(10,4); zeros(4,14)];

Aa_lr    = [A_val + dAc_lr, B_val; zeros(4,10), zeros(4,4)];
H_lr     = [Aa_lr, [B_val; zeros(4,4)]; zeros(4,14), zeros(4,4)] * Ts;
M_exp_lr = expm(H_lr);
dAd_lr_full = M_exp_lr(1:14, 1:14) - Ad_val;
dAd_lr_val  = [dAd_lr_full(1:10, 1:10), zeros(10,4); zeros(4,14)];

Aa_b    = [A_val + dAc_b, B_val; zeros(4,10), zeros(4,4)];
H_b     = [Aa_b, [B_val; zeros(4,4)]; zeros(4,14), zeros(4,4)] * Ts;
M_exp_b = expm(H_b);
dAd_b_full = M_exp_b(1:14, 1:14) - Ad_val;
dAd_b_val  = [dAd_b_full(1:10, 1:10), zeros(10,4); zeros(4,14)];
    
    M_val          = M_func(R_w_ac, R_l_ac, p(1), p(5), p(2), p(6), p(3), p(7), l_c_ac, ...
                            m_w_ac, m_l_ac, m_b_ac, I_w_ac, p(4), p(8), I_b_ac, I_z_ac, g_ac);
    G_val          = G_func(R_w_ac, R_l_ac, p(1), p(5), p(2), p(6), p(3), p(7), l_c_ac, ...
                            m_w_ac, m_l_ac, m_b_ac, I_w_ac, p(4), p(8), I_b_ac, I_z_ac, g_ac);
    B_tau_val      = B_tau_func(R_w_ac, R_l_ac, p(1), p(5), p(2), p(6), p(3), p(7), l_c_ac, ...
                                m_w_ac, m_l_ac, m_b_ac, I_w_ac, p(4), p(8), I_b_ac, I_z_ac, g_ac);
    B_tau_val      = B_tau_func(R_w_ac, R_l_ac, p(1), p(5), p(2), p(6), p(3), p(7), l_c_ac, ...
                            m_w_ac, m_l_ac, m_b_ac, I_w_ac, p(4), p(8), I_b_ac, I_z_ac, g_ac);

% =========================================================
% 【MDO Pitch共模修复】
% B_tau 的腿自身方程行(行1,2)对 T_bl/T_br 符号是 -1
% Pitch 方程行(行4)对 T_bl/T_br 符号是 +1
% 导致 pinv 将 Pitch 扰动差模地分配给两腿，产生劈叉
% 修复：对 pinv 的结果，强制 Pitch 列(第5列)对腿输出行(第3,4行)取平均
% 此操作不影响 B_tau_val 本身，不影响 LQR/MPC/其他通道
% =========================================================
B_tau_pinv_val = pinv(B_tau_val);
avg_pitch_to_legs = (B_tau_pinv_val(3, 5) + B_tau_pinv_val(4, 5)) / 2.0;
B_tau_pinv_val(3, 5) = avg_pitch_to_legs;  % T_bl 行，Pitch扰动列 → 强制共模
B_tau_pinv_val(4, 5) = avg_pitch_to_legs;  % T_br 行，Pitch扰动列 → 强制共模
    
    % =========================================================================
    % 【神级推演重构】：绝对纯净的约束 SSTO 映射 (杜绝 Null-Space 空间爆炸)
    % 强制要求：位置偏移=0, 速度=0, 轮毂力矩=0。
    % 只允许 腿角(索引5,7) 和 腿力矩(索引3,4) 参与方程 (I - Ad)*x - Bd*u = Bd*D
    % =========================================================================
    
    % 提取核心 10x10 的 Ad 和 10x4 的 Bd
    Ad_core = Ad_val(1:10, 1:10);
    Bd_core = Bd_val(1:10, 1:4);
    
    I_minus_Ad = eye(10) - Ad_core;
    
    % 提取允许自由改变的列：左腿角(5), 右腿角(7), -左腿力矩(3), -右腿力矩(4)
    M_core = [I_minus_Ad(:, 5), I_minus_Ad(:, 7), -Bd_core(:, 3), -Bd_core(:, 4)];
    
    % 利用最小二乘求解严格受限的流形映射: M_core * [theta_l; theta_r; tau_l; tau_r] = Bd_core * D_steady
    K_map = pinv(M_core) * Bd_core; 
    
    % 重组装回 10x4 和 4x4 的全尺寸矩阵，其余项绝对为 0！
    M_x_ss = zeros(10, 4);
    M_x_ss(5, :) = K_map(1, :); % 只映射左腿角度目标
    M_x_ss(7, :) = K_map(2, :); % 只映射右腿角度目标
    
    M_u_ss = zeros(4, 4);
    M_u_ss(3, :) = K_map(3, :); % 只映射左腿保底力矩
    M_u_ss(4, :) = K_map(4, :); % 只映射右腿保底力矩
    % =========================================================================
    results(idx, :) = [p(1), p(5), ...
                       reshape(K_hard_val',     1, 40), ...
                       reshape(K_soft_val',     1, 40), ...
                       reshape(P_hard_val',     1, 100), ...
                       reshape(P_soft_val',     1, 100), ...
                       reshape(Ad_val',         1, 196), ...
                       reshape(Ad_pitch_val',   1, 196), ...
                       reshape(Bd_val',         1, 56),  ...
                       reshape(dAd_ll_val',     1, 196), ...
                       reshape(dAd_lr_val',     1, 196), ...
                       reshape(dAd_b_val',      1, 196), ...
                       reshape(M_val',          1, 25),  ...
                       reshape(G_val',          1, 25),  ...
                       reshape(B_tau_val',      1, 20),  ...
                       reshape(B_tau_pinv_val', 1, 20),  ...
                       reshape(M_x_ss',         1, 40),  ... % 新增 SSTO X 映射 (10x4)
                       reshape(M_u_ss',         1, 16)];     % 新增 SSTO U 映射 (4x4)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%Step 4：分区域拟合%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Step 4: 执行 4 区域拟合 (Poly22)...\n');
L_mid = (min(Leg_data_l(:,1)) + max(Leg_data_l(:,1))) / 2;
X = results(:,1); Y = results(:,2);
idx_sets     = {find(X<=L_mid & Y<=L_mid), find(X>L_mid & Y<=L_mid), ...
                find(X<=L_mid & Y>L_mid),  find(X>L_mid & Y>L_mid)};
Region_Names = {'Reg1', 'Reg2', 'Reg3', 'Reg4'};
Output_Data  = struct();

% 【修复后】的代码：
offsets = [2,   42,   82,  182,  282,  478,  674,  730,  926, 1122, 1318, 1343, 1368, 1388, 1408, 1448];
counts  = [40,  40,  100,  100,  196,  196,   56,  196,  196,  196,   25,   25,   20,   20,   40,   16];
fields  = {'K_hard','K_soft','P_hard','P_soft','Ad','Ad_pitch','Bd', ...
           'dAd_ll','dAd_lr','dAd_b','M','G','B_tau','B_tau_pinv', 'M_x_ss', 'M_u_ss'};

for r = 1:4
    local_X   = X(idx_sets{r});
    local_Y   = Y(idx_sets{r});
    local_Res = results(idx_sets{r}, :);
    for m = 1:numel(fields)
        coeffs_mat = zeros(counts(m), 6);
        for k = 1:counts(m)
            z = local_Res(:, offsets(m) + k);
            if range(z) < 1e-10
                c = mean(z); coeffs = [c, 0, 0, 0, 0, 0];
            else
                f = fit([local_X, local_Y], z, 'poly22');
                coeffs = coeffvalues(f);
            end
            if length(coeffs) < 6
                t = zeros(1,6); t(1:length(coeffs)) = coeffs; coeffs = t;
            end
            coeffs_mat(k,:) = coeffs;
        end
        Output_Data.(Region_Names{r}).(fields{m}) = coeffs_mat;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%Step 5：生成头文件%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Step 5: 正在生成 Auto_iLQR_MDO_Data.h ...\n');
fid = fopen('Auto_iLQR_MDO_Data.h', 'w');
fprintf(fid, '#ifndef __AUTO_ILQR_MDO_DATA_H__\n#define __AUTO_ILQR_MDO_DATA_H__\n\n');
fprintf(fid, '// 自动生成：极限融合版（微分方程基石 + 拉格朗日非线性补丁 + SSTO流形映射）\n\n');
fprintf(fid, 'inline const float L_MID_CONST = %.5f;\n\n', L_mid);
fprintf(fid, '// ================= [ Q/R 权重配置 ] =================\n');
q_h = diag(lqr_Q_hard); q_s = diag(lqr_Q_soft);
r_h = diag(lqr_R_hard); r_s = diag(lqr_R_soft);
fprintf(fid, 'inline const float Q_HARD_DIAG[10] = {');
for i=1:10, fprintf(fid,'%.6f',q_h(i)); if i<10, fprintf(fid,', '); end; end
fprintf(fid, '};\n');
fprintf(fid, 'inline const float R_HARD_DIAG[4]  = {');
for i=1:4,  fprintf(fid,'%.6f',r_h(i)); if i<4,  fprintf(fid,', '); end; end
fprintf(fid, '};\n');
fprintf(fid, 'inline const float Q_SOFT_DIAG[10] = {');
for i=1:10, fprintf(fid,'%.6f',q_s(i)); if i<10, fprintf(fid,', '); end; end
fprintf(fid, '};\n');
fprintf(fid, 'inline const float R_SOFT_DIAG[4]  = {');
for i=1:4,  fprintf(fid,'%.6f',r_s(i)); if i<4,  fprintf(fid,', '); end; end
fprintf(fid, '};\n\n');

fprintf(fid, '// ================= [ 拟合系数矩阵 ] =================\n');
maps = {
    'K_hard',      'k_hard_fit_coefficients',      40;
    'K_soft',      'k_soft_fit_coefficients',      40;
    'P_hard',      'p_hard_fit_coefficients',     100;
    'P_soft',      'p_soft_fit_coefficients',     100;
    'Ad',          'ad_fit_coefficients',          196;
    'Ad_pitch',    'ad_pitch_fit_coefficients',    196;
    'Bd',          'bd_fit_coefficients',           56;
    'dAd_ll',      'dad_ll_fit_coefficients',      196;
    'dAd_lr',      'dad_lr_fit_coefficients',      196;
    'dAd_b',       'dad_b_fit_coefficients',       196;
    'M',           'm_fit_coefficients',            25;
    'G',           'g_fit_coefficients',            25;
    'B_tau',       'btau_fit_coefficients',         20;
    'B_tau_pinv',  'btau_pinv_fit_coefficients',    20;
    'M_x_ss',      'm_x_ss_fit_coefficients',       40;  % SSTO X
    'M_u_ss',      'm_u_ss_fit_coefficients',       16   % SSTO U
};
for m = 1:size(maps, 1)
    fname = maps{m,1}; cname = maps{m,2}; rows = maps{m,3};
    fprintf(fid, '// %s (%d elements)\n', fname, rows);
    fprintf(fid, 'inline const float %s[4][%d][6] = {\n', cname, rows);
    for r = 1:4
        data = Output_Data.(Region_Names{r}).(fname);
        fprintf(fid, '    { // Region %d\n', r);
        for ii = 1:rows
            fprintf(fid, '        {%.6f, %.6f, %.6f, %.6f, %.6f, %.6f}', data(ii,:));
            if ii < rows, fprintf(fid, ',\n'); else, fprintf(fid, '\n'); end
        end
        if r < 4, fprintf(fid, '    },\n'); else, fprintf(fid, '    }\n'); end
    end
    fprintf(fid, '};\n\n');
end
fprintf(fid, '#endif\n');
fclose(fid);
fprintf('完美收工！SSTO 流形映射已成功编译进头文件！\n');
toc