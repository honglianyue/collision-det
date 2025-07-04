
run("robot_dh_modified.m");
% 定义质心位置（单位：mm）
% % 2. 连杆重心位置 (mm)，每个连杆三个坐标
% link_cogs = {
%    [0.022, -8.728, 225.433],    % 连杆1: [Xebar, Yebar, Zebar] ×1000
%    [18.130, -70.222, 142.542],  % 连杆2
%    [25.017, 11.409, 40.667],    % 连杆3
%    [-0.0255, -10.923, -71.267],  % 连杆4
%    [0.003, 29.173, 7.019],      % 连杆5
%    [0.073, 0.079, -44.853]      % 连杆6
%    };
% 
% % 3. 连杆惯性矩阵参数 (kg·mm²)
% link_inertia = {
%    [21369.310, 20930.730, 822620.660, 0.113, -3.834, -1763.079],  % 连杆1: [Ixc, Iyc, Izc, Ixyc, Ixzc, Iyzc] ×10⁶
%    [63320.810, 881836.850, 9401.950, 543.525, 8926.120, -320.956], % 连杆2
%    [6588.510, 477810.470, 3388.610, -533.908, 1729.640, -819.081], % 连杆3
%    [13319.850, 12519.890, 231678.280, -1.850, 0.867, -1438.940],   % 连杆4
%    [5032.630, 231504.420, 3944.800, -6.075, -1.293, -331.648],    % 连杆5
%    [997.020, 1002.530, 229337.950, -1.376, 0.897, 1.581]          % 连杆6
%    };
% 2. 连杆重心位置 (mm)，每个连杆三个坐标
link_cogs = {
    [0.022, -8.728, 225.433],     % 连杆1
    [19.368, -68.585, 146.373],   % 连杆2
    [24.115, 12.803, 39.802],     % 连杆3
    [-0.025, -10.923, -71.267],   % 连杆4
    [0.003, 29.173, 7.019],       % 连杆5
    [0.073, 0.079, -44.853]       % 连杆6
};

% 3. 连杆惯性矩阵参数 (kg·mm²)
link_inertia = {
    [21369.338, 20930.725, 822620.658, 0.113, -3.834, -1763.079],  % 连杆1
    [63295.898, 881968.783, 9116.219, 524.974, 9061.836, -171.482], % 连杆2
    [6774.086, 825626.587, 3518.156, -575.854, 1770.774, -902.096], % 连杆3
    [13319.852, 12519.892, 231678.281, -1.850, 0.867, -1438.940],   % 连杆4
    [5032.625, 231504.418, 3944.803, -6.075, -1.293, -331.648],     % 连杆5
    [997.016, 1002.530, 229337.948, -1.376, 0.897, 1.581]           % 连杆6
};
% 假设 T0i 和 T0ii 是 cell 数组，保存了每个连杆的新旧坐标系变换矩阵
% T0i{i} = 新坐标系 {i} 相对于基坐标系的变换矩阵
% T0ii{i} = 旧坐标系 {i} 相对于基坐标系的变换矩阵

n_links = length(link_cogs);  % 连杆数量（如 6）

% 初始化结果容器
new_link_cogs = cell(1, n_links);
new_link_inertia = cell(1, n_links);

TC0i = cell(1, n_links);  % 变换矩阵
for i = 1:n_links
    
    % Step 1: 获取新旧坐标系的变换矩阵
    T_new = T0i{i};      % 新坐标系相对于基坐标系
    T_old = T0ii{i};     % 旧坐标系相对于基坐标系
    
    % Step 2: 计算从旧坐标系到新坐标系的变换矩阵
    T_new_from_old = T_new \ T_old;
    
    TC0i{i} = T_new_from_old;  % 保存变换矩阵
    % Step 3: 提取旋转矩阵 R
    R = T_new_from_old(1:3, 1:3);
    
    % Step 4: 转换质心位置
    cog_old = link_cogs{i};       % 旧坐标系下的质心位置
    cog_old_homogeneous = [cog_old()'; 1];
    cog_new_homogeneous = T_new_from_old * cog_old_homogeneous;
    cog_new = cog_new_homogeneous(1:3)';
    new_link_cogs{i} = cog_new;
    % Step 5: 转换惯性矩阵
    inertia_old_vec = link_inertia{i};  % 旧惯性矩阵参数（向量形式）
    
    % 将向量形式转换为惯性矩阵
    I_old = [
        inertia_old_vec(1), inertia_old_vec(4), inertia_old_vec(5);
        inertia_old_vec(4), inertia_old_vec(2), inertia_old_vec(6);
        inertia_old_vec(5), inertia_old_vec(6), inertia_old_vec(3)
    ];
    
    % 应用坐标变换
    I_new = R * I_old * R';
    
    % 重新整理成向量形式 [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    inertia_new_vec = [
        I_new(1,1), ...
        I_new(2,2), ...
        I_new(3,3), ...
        I_new(1,2), ...
        I_new(1,3), ...
        I_new(2,3)
    ];
    
    % 保存到新惯性矩阵数组
    new_link_inertia{i} = inertia_new_vec;
    
end

% 显示结果示例
disp('新坐标系下的质心位置:');
for i = 1:n_links
    fprintf('连杆 %d: [%f, %f, %f]\n', i, new_link_cogs{i}(1), new_link_cogs{i}(2), new_link_cogs{i}(3));
end

disp('新坐标系下的惯性矩阵参数:');
for i = 1:n_links
    fprintf('连杆 %d: [%f, %f, %f, %f, %f, %f]\n', i, new_link_inertia{i}(1), ...
                                                       new_link_inertia{i}(2), ...
                                                       new_link_inertia{i}(3), ...
                                                       new_link_inertia{i}(4), ...
                                                       new_link_inertia{i}(5), ...
                                                       new_link_inertia{i}(6));
end


save('new_link.mat', 'new_link_cogs', 'new_link_inertia');