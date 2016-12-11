% Tuning Curves

% INPUT % % % % % % % % % % % % % % % %
% read in a file formatted:
% the state variables represent the starting state that the sim was run
% from
% only run this with data from a single state, for now
% iter, depth, exp const, x_car, y_car, x_amb, y_amb, vx_amb, vy_amb, discounted reward

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% comparing DPW and Vanilla

% import the data

dataDPW = importfile('exp_const_v_utility_final_DPW_c1_1000.csv');
dataVan = importfile('threevanilla.csv');

iter = dataDPW(1,1);
depth = dataDPW(1,2);

figure(); hold on;
x_subset1 = dataDPW(:,3);
y_subset1 = dataDPW(:,10);
x_subset2 = dataVan(:,3);
y_subset2 = dataVan(:,10);
plot(x_subset1,y_subset1, 'x-', x_subset2, y_subset2,'x-')

ylabel('Total Discounted Reward');
xlabel('Exploration Constant C_P');
legend('DPW','Vanilla');
t_str = sprintf('Discounted Reward vs. Exploration Constant, %.0f Iterations, Depth %.1f',iter,depth);
title(t_str);

