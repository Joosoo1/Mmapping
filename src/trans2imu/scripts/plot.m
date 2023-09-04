% 从文本文件读取数据，假设数据以空格分隔
data = dlmread('data.txt');

% 提取时间间隔，假设时间间隔为0.001秒
time = (0:size(data, 1)-1) * 0.001;

% 提取第2列、3列、4列的数据
column2 = data(:, 2);
column3 = data(:, 3);
column4 = data(:, 4);

% 创建一个新的图形窗口
figure;

% 绘制第2列的数据
subplot(3, 1, 1);
plot(time, column2, 'r-', 'LineWidth', 2);
xlabel('时间 (秒)');
ylabel('数据');
title('第2列数据随时间变化');
grid on;

% 绘制第3列的数据
subplot(3, 1, 2);
plot(time, column3, 'g-', 'LineWidth', 2);
xlabel('时间 (秒)');
ylabel('数据');
title('第3列数据随时间变化');

% 绘制第4列的数据
subplot(3, 1, 3);
plot(time, column4, 'b-', 'LineWidth', 2);
xlabel('时间 (秒)');
ylabel('数据');
title('第4列数据随时间变化');

% 调整子图之间的间距
spacing = 0.05;
subplotSpacing = 0.08;
subplotPosition = 0.9 / 3;
set(gcf, 'Position', [100, 100, 800, 600]);

% 调整子图位置
pos = get(gca, 'Position');
pos(2) = subplotPosition * 2 + subplotSpacing * 2;
set(gca, 'Position', pos);

pos = get(gca, 'Position');
pos(2) = subplotPosition + subplotSpacing;
set(gca, 'Position', pos);

