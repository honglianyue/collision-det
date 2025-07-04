% 运行参数估计的主脚本
% 作者: honglianyue
% 日期: 2025-06-23

% 假设有N组历史数据，6个轴
alpha1 = zeros(6,1);
gamma = zeros(6,1);
epsilon_fr = zeros(6,1);

N = size(ExternalTauCalculated,1);
for axis = 1:6
    Y = ExternalTauCalculated(index:total_steps,axis); % N×1
    X = [ones(total_steps-index+1,1),H_T(index:total_steps,axis),K1*INTEGRAL_TERM(index:total_steps,axis)]; % N×1
    params = X \ Y; % params = [alpha; gamma; epsilon_fr]
    alpha1(axis) = params(1);
    gamma(axis) = params(2);
    epsilon_fr(axis) = params(3);
end

% 保存参数
save('threshold_parameters_10_08_22.mat', 'alpha1', 'gamma', 'epsilon_fr');
for axis = 1:6
    Y = ExternalTauCalculated(index:total_steps,axis); % N×1
    X = [H_T(index:total_steps,axis),K1*INTEGRAL_TERM(index:total_steps,axis)]; % N×1
    params = X \ Y; % params = [alpha; gamma; epsilon_fr]
    gamma(axis) = params(1);
    epsilon_fr(axis) = params(2);
end
save('threshold_parameters_10_08_22_1.mat',  'gamma', 'epsilon_fr');
% for axis = 1:6
%     Y = ExternalTauCalculated(index:total_steps,axis); % N×1
%     X = [ones(total_steps-index+1,1),H_T(index:total_steps,axis)]; % N×2
%     params = X \ Y; % params = [alpha; gamma; epsilon_fr]
%     alpha1(axis) = params(1);
%     gamma(axis) = params(2);
% end
% save('threshold_parameters1.mat', 'alpha1', 'gamma');