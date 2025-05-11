%% Load Data
Data = xlsread("Battery_Parameters.xlsx");

%% Name the data
SOC = Data(:,1);
OCV = Data(:,2);
R_Charge = Data(:,3);
R_Discharge = Data(:,4);

%% Plot Data
plot(SOC, OCV);
figure
plot(SOC, R_Charge);
figure
plot(SOC, R_Discharge);

%% Simulate
I = 2.3 % Current [A]
Cn = 2.3 * 3600 % Capacity [A/s]
Sim_Time = 3600;
sim("Project5.slx")