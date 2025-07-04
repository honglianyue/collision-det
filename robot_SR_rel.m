% 清除命令行窗口
clc
% 关闭所有图形窗口
close all
%机器人参数配置
run("robot_sr_para.m")
load('threshold_parameters_09_29_23.mat');%未考虑摩擦力因素
%% 开始算法流程

% 加载实机数据：
filename = '2025_07_03_09_25_10.data';
[data, joint_data, collision_data] = read_robot_data(filename);

%起始点
start =1;

% % 2. 获取数据长度和采样时间
 total_steps = size(joint_data.position, 1)-start;
 % total_steps = 10000 ;
DeltaT = 0.001; % 采样时间1ms

% 滑模控制增益
S1=100; T1=sqrt(S1);
S2=2000 ; T2=sqrt(S2)*2;
% k1=30;k2=60;k3=300;
% beta = 0.3;

% 动量观测器控制增益
gain=100*diag([1,1,1,1,1,1]);
beta1 = 2000;%2100 NAN
beta2 = 500;
% 动量观测器增益
GainInv=((eye(6)+gain*DeltaT))\gain;
% 存储数据
% ExternalTauCalculated：计算的外部关节力矩
ExternalTauCalculated = zeros(total_steps, 6);
% ForceCalculated：计算的外部力
ForceCalculated = zeros(total_steps, 6);
% ExternalTauCalculated2：计算的外部关节力矩2
ExternalTauCalculated2 = zeros(total_steps, 6);
% ForceCalculated2：计算的外部力2
ForceCalculated2 = zeros(total_steps, 6);

% 摩擦力矩阵
Ff1 = zeros(total_steps, 6);
Ff2 = zeros(total_steps, 6);
% 初始化时间统计变量
sliding_mode_time = 0;
generalized_momentum_time = 0;

% 3. 使用实机数据替换仿真数据
Q_sampled = joint_data.position(start:end,:);      % 关节位置
QD_sampled = joint_data.velocity(start:end,:);     % 关节速度
TAU_applied = joint_data.feedbacktrq(start:end,:); % 驱动力矩
TauExternal = collision_data.torque(start:end,:);  % 实际碰撞力矩
Friction = joint_data.frictions(start:end,:);

% 4. 初始化观测器
index =1;
samples1 =60;  % 滑动窗口大小
sigma = zeros(6,1);
sigma2 = zeros(6,1);
r = zeros(6,1);
r2 = zeros(6,1);
% time：时间
t0 = 0;
time = zeros(total_steps, 1);

%时变阈值
collision_time =zeros(total_steps, 6); % 碰撞时间记录
upper_thresholds = zeros(total_steps, 6); % 上阈值
lower_thresholds = zeros(total_steps, 6); % 下阈值

H_T = zeros(total_steps, 6); % h(t)的值
INTEGRAL_TERM = zeros(total_steps, 6); % 积分项

% 残差滤波器参数设置
fc_residual = 10;    % 截止频率 (Hz)
Ts = DeltaT;         % 采样周期
fc_tau = 10;
% 初始化滤波器历史值
tau_filtered_prev = zeros(6, 1);
TauAppliedFiltered = zeros(total_steps, 6);
% 初始化残差滤波器历史值
r_filtered = zeros(6,1);  % 初始值
r_filtered2 = zeros(6,1);  % 初始值
% 存储滤波后的残差
ResidualFiltered = zeros(total_steps, 6);
ResidualFiltered2 = zeros(total_steps, 6);
mm = zeros(total_steps, 1);
%% 5. 在循环中使用实机数据
while(index <= total_steps)
   
   % 显示当前时间
   disp(['time instant: ', num2str(t0)]);
   % 获取当前力矩数据
   tau_current = TAU_applied(index, :)';
   
   % 应用低通滤波器
   tau_filtered = lowPassFilter(tau_current, tau_filtered_prev, fc_tau, Ts);
   
   % 存储滤波后的力矩数据
   TauAppliedFiltered(index, :) = tau_filtered';
   
   % 更新历史值
   tau_filtered_prev = tau_filtered;
   
   % 获取当前状态
   q0 = Q_sampled(index,:)';
   qd0 = QD_sampled(index,:)';
   
   % 计算动力学参数
   g = robot.gravload(q0')';
   M = robot.inertia(q0');
   C = robot.coriolis(q0', qd0');
   J = robot.jacob0(q0');
   pinvJ = pinv(J);
   % 设置速度阈值
   
   velocity_threshold = 0.01;  % 速度阈值，单位：rad/s
   
   % 计算摩擦力矩，考虑速度阈值
   friction1 = zeros(size(qd0'));
   friction2 = zeros(size(qd0'));
   for i = 1:length(qd0)
      if abs(qd0(i)) < velocity_threshold
         % 速度小于阈值时，只考虑库仑摩擦
         friction1(i) = friction_params.Coulomb(i) * sign(qd0(i));
      else
         % 速度大于阈值时，考虑所有摩擦分量
         friction1(i) = friction_params.viscous(i) * qd0(i ) + friction_params.Coulomb(i) * sign(qd0(i)) + friction_params.offset(i);
      end
   end
   Ff1(index,:)=friction1;
   % 存储中间计算结果
   B_sampled{index} = M;
   c_sampled{index} = C; % 将C赋值给S_sampled的第index个元素
   g_sampled{index} = g; % 将g赋值给g_sampled的第index个元素
   h_sampled{index} = C'*qd0 - g;
   BQD{index} = M *QD_sampled(index, :)';
   K1=0.5;% 将M乘以QD_sampled的第index行的转置赋值给BQD的第index个元素
   %% 二阶线性滑模观测器
   if index>samples1
      dsigma = zeros(6,1);
      sumTau = 0;
      sumH = 0;
      sumRes = 0;
      integral_term = zeros(6,1);
      integral_term_h = zeros(6,1);
      p0 = 0;
      for tt = 1:samples1+1
         t=index-samples1+tt-1;
         p=B_sampled{t}*QD_sampled(t, :)';
         % 计算指数衰减的积分项
         current_time = (samples1+1)*DeltaT;
         tau_fr = Friction(t,:)';%需要准确的摩擦力模型
         integral_term = integral_term + exp(-K1*(current_time-tt*DeltaT)) .* tau_fr * DeltaT;
         % 计算h(t)所需的积分项
         integral_term_h = integral_term_h + exp(-K1*(current_time-tt*DeltaT)) .* (K1*p + h_sampled{t}) * DeltaT;
         if tt == 1
            p_hat = p;
         else
            p_error = p_hat - p;
            bounds = 0.001;
            signP = p_error ./ (abs(p_error) + bounds);  % 平滑边界层过渡
            p_norm = sqrt(abs(p_error));
            dp_hat = double(TAU_applied(t,:)' + h_sampled{t} + sigma - T2*p_error - T1*p_norm.*signP);
            dsigma = double(-S1*signP - S2*p_error);
               % dp_hat = double(TAU_applied(t,:)' + h_sampled{t} + sigma - beta1*p_error);
               % dsigma = double(-beta2*p_error);
               % mm(index) = 1; 
            % % 四阶龙格库塔法更新
            k1 = DeltaT * dp_hat;
            k2 = DeltaT * (dp_hat + 0.5*k1);
            k3 = DeltaT * (dp_hat + 0.5*k2);
            k4 = DeltaT * (dp_hat + k3);
            p_hat = p_hat + (k1 + 2*k2 + 2*k3 + k4) / 6; % 更新p_hat
            k1 = DeltaT * dsigma;
            k2 = DeltaT * (dsigma + 0.5*k1);
            k3 = DeltaT * (dsigma + 0.5*k2);
            k4 = DeltaT * (dsigma + k3);
            sigma = sigma + (k1 + 2*k2 + 2*k3 + k4) / 6; % 更新sigma
         end
      end
      % 保存估计结果
      % ExternalTauCalculated(index,:) = double(sigma');
      ExternalTauCalculated(index,:) = double(sigma');
      ForceCalculated(index,:) = pinvJ * sigma;
      r_filtered = lowPassFilter(sigma, r_filtered, fc_residual, Ts);
      sigma  = r_filtered;
      % 存储滤波后的残差
      ResidualFiltered(index, :) = r_filtered';
      sliding_mode_time = sliding_mode_time + toc;  % 累加计算时间
      
      % 计算b2的边界 论文公式24 26
      h_t = K1*(p-integral_term_h); % 计算h(t)
      H_T(index,:) = h_t'; % 存储h(t)的值
      INTEGRAL_TERM(index,:) = integral_term'; % 存储积分项
      % ths = [4,7,3,1.2,0.8,1.2]';
      ths = [3.5,3.5,2.8,1.2,0.6,0.5]';%不考虑摩擦力
      b2H = alpha1+ths + gamma.*h_t + epsilon_fr.*K1.*integral_term;
      b2L = alpha1-ths + gamma.*h_t  + epsilon_fr.*K1.*integral_term;
      %储存阈值上下界
      upper_thresholds(index,:) = b2H';
      lower_thresholds(index,:) = b2L';
      %分别判断每个轴上的外力是否超过阈值
      for i = 1:6
         if ResidualFiltered(index,i) > b2H(i)
            fprintf('外力超过上限: %d轴, 外力: %.4f, 上限: %.4f\n', i, ResidualFiltered(index,i), b2H(i));
            collision_time(index,i) = 10; % 记录碰撞时间
         elseif ResidualFiltered(index,i) < b2L(i)
            fprintf('外力超过下限: %d轴, 外力: %.4f, 下限: %.4f\n', i, ResidualFiltered(index,i), b2L(i));
            collision_time(index,i) = 10; % 记录碰撞时间
         end
      end
      %% 动量法外力观测
      tic;
      % for tt = 1:samples1+1
      %    t=index-samples1+tt-1;
      %    p=B_sampled{t}*QD_sampled(t, :)';
      %    if tt == 1
      %       p_hat2 = p;
      %    else
      %       bounds = 0.001;
      %       %扩展状态动量观测器
      %       p_error2 = p_hat2 - p;
      %       dp_hat2 = double(TAU_applied(t,:)' + h_sampled{t} + sigma2 - beta1*p_error2);
      %       dsigma2 = double(-beta2*p_error2);
      %       p_hat2 = p_hat2 + DeltaT*dp_hat2;
      %       sigma2  = sigma2 + DeltaT*dsigma2;
      %    end
      % end
      % % 保存估计结果
      % ExternalTauCalculated2(index,:) = double(sigma2');
      % ForceCalculated2(index,:) = pinvJ * sigma2;
      % r_filtered2 = lowPassFilter(sigma2, r_filtered2, fc_residual, Ts);
      % sigma2 = r_filtered2;
      % % 存储滤波后的残差
      % ResidualFiltered2(index, :) = r_filtered2';
      for tt = 1:samples1+1
            t=index-samples1+tt-1;
            h=h_sampled{t};
            sumTau = sumTau + TAU_applied(t,:)';
            sumH = sumH + h;
            if tt == 1
               p0 = B_sampled{t}*QD_sampled(t, :)';
            else
               r = GainInv* ((BQD{t} - p0) - (sumTau + sumH + sumRes)*DeltaT);
               sumRes = sumRes + r; % 残差积分
            end
     end
     ExternalTauCalculated2(index,:) = r';
     ForceCalculated2(index,:) = pinvJ *r;
     generalized_momentum_time = generalized_momentum_time + toc;
   end
   index = index + 1;
   t0=t0+DeltaT;
   time(index)=t0;
end

run("huitu.m");

function y_filtered = lowPassFilter(y, y_prev, fc, Ts)
% LOWPASSFILTER 实现一阶低通滤波
% 输入:
%   y       - 当前时刻输入信号 (标量或向量)
%   y_prev  - 上一时刻输出值
%   fc      - 截止频率 (Hz)
%   Ts      - 采样周期 (s)
%
% 输出:
%   y_filtered - 滤波后的输出信号
alpha = 2 * pi * fc * Ts / (1 + 2 * pi * fc * Ts);
y_filtered = alpha * y + (1 - alpha) * y_prev;
end

