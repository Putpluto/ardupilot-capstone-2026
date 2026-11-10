clc
clear
close all

% ArduCopter SITL example vehicle - Updated for 7-inch, 6S 1300KV
% Custom AUW: 810g with 5.2Ah Battery
% setup the motors
% locations (xyz) in m (Wheelbase ~300mm for 7-inch)
frame_size = 300; 
motor(1).location = [[sind(45),cosd(45)]*frame_size*0.5,0] * 0.001; % front right
motor(2).location = [[sind(135),cosd(135)]*frame_size*0.5,0] * 0.001; % rear right
motor(3).location = [[sind(225),cosd(225)]*frame_size*0.5,0] * 0.001; % rear left
motor(4).location = [[sind(315),cosd(315)]*frame_size*0.5,0] * 0.001; % front left

% PWM output to use
motor(1).channel = 1;
motor(2).channel = 4;
motor(3).channel = 2;
motor(4).channel = 3;

% rotation direction: 1 = cw, -1 = ccw
motor(1).direction = -1;
motor(2).direction = 1;
motor(3).direction = -1;
motor(4).direction = 1;

% motor properties
electrical.kv = 1300; % (rpm/volt)
electrical.no_load_current = [0.7,10]; % (A) @ (V)
electrical.resistance = 0.115; % (ohms)

% ESC properties
esc.resistance = 0.01; % (ohms)

% Propeller properties (7-inch prop)
prop.diameter = 177.8 * 0.001; % (m) - 7 inches
prop.pitch = 101.6 * 0.001; % (m) - 4 inches
prop.num_blades = 2;
prop.PConst = 1.13;
prop.TConst = 1;
prop.mass = 6.5 * 0.001; % (kg)
prop.inertia = (1/12)*prop.mass*prop.diameter^2; % rotational inertia

% assign properties to motors
for i = 1:4
    motor(i).electrical = electrical;
    motor(i).esc = esc;
    motor(i).prop = prop;
end

% Setup battery 
battery.voltage = 6*4.2; % (volts) - 6S
battery.resistance = 0.0034; % (ohms)
battery.capacity = 5.2; % (Ah) - AS REQUESTED

% Add all to vehicle
copter.motors = motor;
copter.battery = battery;
copter.mass = 0.81; % (kg) - AS REQUESTED (810 grams)

% Re-calculate inertia for the 300mm frame
inertia = (2/5) * copter.mass * ( (frame_size/1000) * 0.2 )^2; 
copter.inertia = diag(ones(3,1)*inertia);
copter.cd = [0.5;0.5;0.5];
copter.cd_ref_area = [1;1;1] * pi * ( (frame_size/1000) * 0.5 )^2;

save('Quad7Inch_6S_CustomAUW','copter')

% Plot motor curves
max_power = 800; 
battery.voltage = battery.voltage * 0.50; 
Kt = 1/(electrical.kv * ((2*pi)/60) ); 

% plot the current
amps = 0:0.1:max_power/battery.voltage;
power_in = amps * battery.voltage;
copper_drop = amps * electrical.resistance; 
esc_drop = amps * esc.resistance;
ideal_voltage = battery.voltage - copper_drop - esc_drop;
power_out = ideal_voltage .* (amps - electrical.no_load_current(1));
efficiency = power_out ./ power_in;
torque = Kt * amps;
rpm = ideal_voltage * electrical.kv;

% Plot motor characteristics
figure('name',sprintf('motor characteristics at %0.2f volts',battery.voltage))
subplot(2,2,1); hold all; title('RPM'); plot(amps,rpm); xlabel('Current (A)'); ylabel('RPM'); xlim([0,amps(end)]);
subplot(2,2,2); hold all; title('torque'); plot(amps,torque); xlabel('Current (A)'); ylabel('torque (NM)'); xlim([0,amps(end)]);
subplot(2,2,3); hold all; title('power'); plot(amps,power_in); plot(amps,power_out); xlabel('Current (A)'); ylabel('power (W)'); ylim([0,inf]); xlim([0,amps(end)]); legend('Power in','Power out','location','northwest')
subplot(2,2,4); hold all; title('efficiency'); plot(amps,efficiency); xlabel('Current (A)'); ylabel('efficiency (%)'); ylim([0,inf]); xlim([0,amps(end)]);