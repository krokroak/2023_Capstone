%% motor spec 
 BL9_V = 24;
BL9_A = 10;

rated_power = 0.2;  % kw
rated_torque = 0.6272; %[Nm]
motor_length = 0.0725; %m
motor_weight = 1.9;    %kg
rated_speed = 5.655;    %m/s 

poles = 8;
hz = 60;
n1 = 1;
n2 = 50;
a = 1/50;

gearhead_torque = 16.856; %Nm


%% spraket

n = hz * 120/poles;

Torque = 9.55 * (rated_power / n);

%% chain force

chain_pitch = 25.40; %mm
N_teeth = 20;
q_teeth = 60;
rotate_t  = 50;

V = chain_pitch * N_teeth * 50 / 1000;

F = 60 * 0.2/V;
C = 200/25.40 ;
L = (N_teeth + q_teeth)/2 + 2 * C + (((q_teeth - N_teeth)/6.28)^2)/C ;