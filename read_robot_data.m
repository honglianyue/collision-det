function [data, joint_data, collision_data] = read_robot_data(filename)
    % 读取数据文件
    fid = fopen(filename, 'r');
    if fid == -1
        error('无法打开文件: %s', filename);
    end
    
    % 读取表头
    header = fgetl(fid);
    column_names = strsplit(header, '\t');
    
    % 读取数据
    data_cell = textscan(fid, repmat('%f', 1, length(column_names)), 'Delimiter', '\t');
    fclose(fid);
    
    % 转换为矩阵
    data = cell2mat(data_cell);
    
    % 提取关节数据
    joint_data = struct();
    joint_data.position = data(:,1:6);  % 关节位置
    joint_data.velocity = data(:,7:12);    % 关节速度
    joint_data.feedbacktrq = data(:,13:18);   % 反馈力矩
    joint_data.thresold = data(:,19:24); % 固定碰撞检测阈值
    % joint_data.position = data(:,13:18);  % 关节位置
    % joint_data.velocity = data(:,7:12);    % 关节速度
    % joint_data.feedbacktrq = data(:,19:24);   % 反馈力矩

    joint_data.frictions = data(:,31:36);%摩擦力矩
    % 提取碰撞检测数据
    collision_data = struct();
    collision_data.torque = data(:,25:30); % 碰撞检测力矩
    
    % 绘制数据
    plot_robot_data(joint_data, collision_data);
end

function plot_robot_data(joint_data, collision_data)

    % 绘制关节位置
    figure
    for i = 1:6
        subplot(3, 2, i);
        plot(joint_data.position(:, i), 'LineWidth', 1.5);
        hold on;
        title(['关节' num2str(i) '位置']);
        xlabel('时间 (s)');
        ylabel('角度rad)');
        grid on;
    end

    % 绘制关节速度
    figure
    for i = 1:6
        subplot(3, 2, i);
        plot(joint_data.velocity(:, i), 'LineWidth', 1.5);
        hold on;
        title(['关节' num2str(i) '速度']);
        xlabel('时间 (s)');
        ylabel('角速度 (rad/s)');
        grid on;
    end

    % 绘制前馈力矩和电机驱动力矩
    figure
    for i = 1:6
        subplot(3, 2, i);
        plot(joint_data.feedbacktrq(:, i), 'LineWidth', 1.5);
        title(['关节力矩 ' num2str(i)]);
        xlabel('时间 (s)');
        ylabel('力矩 (N·m)');
        legend('传感器力矩');
        grid on;
    end

    figure
    for i = 1:6
         subplot(3,2,i);
         plot(joint_data.frictions(:, i), 'LineWidth', 1.5);
         title(['摩擦力矩 ' num2str(i)]);
         xlabel('时间 (s)');
         ylabel('力矩 (N·m)');
    end
    % 绘制碰撞检测力矩
    figure
    for i = 1:6
        subplot(3, 2, i);
        plot(collision_data.torque(:, i), 'LineWidth', 1.5);
        title(['碰撞检测力矩 ' num2str(i)]);
        xlabel('时间 (s)');
        ylabel('力矩 (N·m)');
        grid on;
    end
    
    
end