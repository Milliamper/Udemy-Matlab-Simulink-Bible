m = 1000;
b = 50;
r = 10; % step input value

s = tf('s'); % transfer function
Plant_TF = 1/(m*s + b);

Kp = 1000; % proportional element
Controller = pid(Kp);

TF_ClosedLoop = feedback(Controller*Plant_TF,1);

%% use Kp equal to 100 and a reference speed of 10m/s
t = 0:0.1:20;
step(r*TF_ClosedLoop,t)
axis([0 20 0 10]) % first two parameters:x | second two parameters: y

%% PID Controller
Kp = 800;
Ki = 40;
Controller = pid(Kp, Ki);

TF_ClosedLoop = feedback(Controller*Plant_TF,1);

step(r*T,t)
axis([0 20 0 10])