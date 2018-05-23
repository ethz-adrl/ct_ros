clear all
close all

load masspointLog.mat

figure()
subplot(2,1,1)
plot(squeeze(t),x(1,:))
xlabel('time')
ylabel('x position')
subplot(2,1,2)
plot(squeeze(t),x(2,:))
xlabel('time')
ylabel('x velocity')

figure()
%plot(squeeze(t(1:end-1)), u(1,:)); hold on;
%plot(squeeze(t(1:end-1)), u(2,:));
subplot(2,1,1)
plot(squeeze(t(1:end-1)), sqrt(u(1,:).^2 + u(2,:).^2));
xlabel('time')
ylabel('absolute force')
subplot(2,1,2)
plot(squeeze(t(1:end-1)), atan(u(2,:)./u(1,:)));
xlabel('time')
ylabel('angle of attack')



