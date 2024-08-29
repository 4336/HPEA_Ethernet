figure(2); clf;
% subplot(3,1,1);
hold on; grid on;
thick = 2;
thin = 0.5;
G = 9.81
arm_length = 0.3; % m
plot(state.stamp/1000000, state.pos*arm_length, 'LineWidth', thick)

% subplot(3,1,2:3); hold on; grid on;

plot(state.stamp/1000000, command.valve2, 'LineWidth', thick)

plot(state.stamp/1000000, state.pressure1/1000, 'LineWidth', thick)

% ylim([-0.1,3])
legend("pos", "valve2", "pres1")
