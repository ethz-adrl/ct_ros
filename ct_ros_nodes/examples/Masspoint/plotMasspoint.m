clear all
close all

load masspointLog.mat

figure()
subplot(2,1,1)
plot(squeeze(t),x(1,:))
ylabel('x position')
subplot(2,1,2)
plot(squeeze(t),x(2,:))
xlabel('time')
ylabel('x velocity')

figure()
subplot(2,2,1)
plot(squeeze(t(1:end-1)), u(1,:)); hold on;
ylabel('F_x');
grid on;
subplot(2,2,3)
plot(squeeze(t(1:end-1)), u(2,:));
ylabel('F_y')
xlabel('time');
grid on;
subplot(2,2,2)
plot(squeeze(t(1:end-1)), sqrt(u(1,:).^2 + u(2,:).^2));
ylabel('|F|')
grid on;
subplot(2,2,4)
plot(squeeze(t(1:end-1)), atan2(u(2,:),u(1,:))); hold on;
plot(squeeze(t(1:end-1)), 3.141*ones(1, length(t)-1), 'k--');
xlabel('time')
ylabel('\theta')
grid on;



