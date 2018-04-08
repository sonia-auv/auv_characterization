clear
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
x11 = ParseData('/home/olavoie/Workspaces/ros_sonia_ws/src/auv_characterization/Matlab/Data/Test 9 Décembre/Numerical data/x_10_1.mat', 0.1,1);
x12 = ParseData('/home/olavoie/Workspaces/ros_sonia_ws/src/auv_characterization/Matlab/Data/Test 9 Décembre/Numerical data/x_10_2.mat', 0.07,1); 

data1      = [x11, x12];
speed1     = filter(b,1, data1);
thrust1    = zeros(1, length(speed1));
thrust1(:) = 1.155118584632874;

x21 = ParseData('/home/olavoie/Workspaces/ros_sonia_ws/src/auv_characterization/Matlab/Data/Test 9 Décembre/Numerical data/x_25_1.mat', 0.2,1); 
x22 = ParseData('/home/olavoie/Workspaces/ros_sonia_ws/src/auv_characterization/Matlab/Data/Test 9 Décembre/Numerical data/x_25_2.mat', 0.2,1); 
x23 = ParseData('/home/olavoie/Workspaces/ros_sonia_ws/src/auv_characterization/Matlab/Data/Test 9 Décembre/Numerical data/x_25_2.mat', 0.2,1); 

data2      = [x21, x22, x23];
speed2     = filter(b, 1, data2);
thrust2    = zeros(1, length(data2));
thrust2(:) = 3.138251304626465;

speed  = [speed1, speed2];
thrust = [thrust1, thrust2];

data = [speed; thrust];
data = conj(data');

vv = data(:, 1) .* data(:, 1);
v  = data(:, 1);

speed = [v, vv];
r = inv(conj(speed') * speed) * conj(speed') * conj(thrust');

function x = ParseData(file_name, threshold, axis) 
 load(file_name);
 LinearSpeed  = LINEAR_SPEED;
 
 windowSize = 5; 
 b = (1/windowSize)*ones(1,windowSize);
 LinearSpeedFilter = filter(b,1, LinearSpeed(:, axis));
 
 t = length(LinearSpeed(:,axis));
 j = 0;
 for i = 1:t
     
     if LinearSpeedFilter(i) >= threshold
         j = j + 1;
         x(j) = LinearSpeedFilter(i);
     end
     
 end
 plot(1:length(x),x)
 hold on 
 plot(1:t,LinearSpeedFilter)
end