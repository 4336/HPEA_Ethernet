figure(1); clf;
subplot(3,1,1); hold on; grid on;
thick = 2;
thin = 0.5;
G = 9.81
plot(state.stamp/1000000, state.pos*0.3, 'LineWidth', thick)

subplot(3,1,2:3); hold on; grid on;
plot(state.stamp/1000000, state.tau*(-0.3), 'LineWidth', thick)
plot(state.stamp/1000000, state.loadcell, 'LineWidth', thin)
plot(state.stamp/1000000, lowpass(state.loadcell, 1, 1000), 'LineWidth', thick)
plot(state.stamp/1000000, state.pressure1/1000, 'LineWidth', thick)
plot(state.stamp/1000000, command.valve1, 'LineWidth', thick)
% plot(state.stamp/1000000, (state.wallstamp - state.pingstamp)/1000,'LineWidth', thick) % ping
% ylim([-0.1,3])
legend()
