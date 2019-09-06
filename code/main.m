% This is the code for Conference paper "Observer-Based Tracking Control for Suppressing Stick-Slip
% Vibration of Drillstring System"
% Author£ºCJ
% Date£º2018/1/19

% This m file is the main function used to calculate the response of
% drill-string subject to active control and bit-rock interaction.
function main 
% clc
% clear all
close all

% Solving state feeedback gain and observer gain
[L,LL] = LQR_PolePlacement;

%%
% simulation time
time = [0 100];

% the total number of state is 15
% x1-x7->original system states
% x8->state of the internal model
% x9-x15->states of observer
x0 = zeros(15,1); 

% Solving the whole system (ODE) via ode45 method
[t,x] = ode45(@drillstring,time,x0,[],L,LL);
            

figure(1)
set(gcf,'Position',[300,300,600,300])
hold on
plot([0,100],[12,12],'r--')
plot(t,x(:,1),'g',t,x(:,4),'b');
legend('Reference','Rotary table speed','Bit speed')
xlabel('Times (s)')
ylabel('Speed (rad/s)')
axis([0 100 -5 20])
grid on
box on

figure(2)
set(gcf,'Position',[300,300,600,300])
plot(t,x(:,4),'b',t,x(:,12),'m');
legend('Bit speed','Observed bit speed')
xlabel('Times (s)')
ylabel('Speed (rad/s)')
axis([0 100 -5 20])
grid on
box on

end