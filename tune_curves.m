% Tuning Curves

% INPUT % % % % % % % % % % % % % % % %
% read in a file formatted:
% the state variables represent the starting state that the sim was run
% from
% only run this with data from a single state, for now
% iter, depth, exp const, x_car, y_car, x_amb, y_amb, vx_amb, vy_amb, discounted reward

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% import the data

LogicalStr = {'false', 'true'};

data = importfile('log_batch_cexp_7.csv');

%depth = 10:5:50;
iter = 200;%:100:500;
const = data(:,3);

figure(); hold on;
for c = const
    %indices = (data(:,3)==c);
    %x_subset = data(indices, 2);
    x_subset = data(:,3);
    %y_subset = data(indices, 10);
    y_subset = data(:,10);
    plot(x_subset,y_subset,'x-')
end

ylabel('Total Discounted Reward');
%xlabel('Depth of MCTS');
%legend('200 iterations', '300 iterations', '400 iterations', '500 iterations');
%title('Tuning depth and iteration parameters');

title('Varying exploration constant');
xlabel('Exp Const');

%% comparing DPW

% import the data

LogicalStr = {'false', 'true'};

dataDPW = importfile('log_batch_dpw.csv');
dataVan = importfile('log_batch_d_10_5_50_iter_200.csv');

depth = 10:5:50;
iter = dataDPW(1,1);%:100:500;
const = dataDPW(1,3); %1e-5:.2:2;

figure(); hold on;
x_subset1 = dataDPW(:,2);
y_subset1 = dataDPW(:,10);
x_subset2 = dataVan(:,2);
y_subset2 = dataVan(:,10);
plot(x_subset1,y_subset1, 'x-', x_subset2, y_subset2,'x-')

ylabel('Total Discounted Reward');
xlabel('Depth of MCTS');
legend('DPW','Vanilla');
t_str = sprintf('Discounted Reward vs. MCTS Depth, %.0f Iterations, Exploration constant %.1f',iter,const);
title(t_str);

