clear
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
x11 = ParseData('/home/olavoie/Workspaces/ros_sonia_ws/src/auv_characterization/Matlab/Data/Test 9 Décembre/Numerical data/y_10_1.mat', 0.1,10,2);
x12 = ParseData('/home/olavoie/Workspaces/ros_sonia_ws/src/auv_characterization/Matlab/Data/Test 9 Décembre/Numerical data/y_10_2.mat', 0.1,10,2); 

data1      = [x11, x12];
speed1     = filter(b,1, data1);
thrust1    = zeros(1, length(speed1));
thrust1(:) = 18.9761;

x02 = ParseData('/home/olavoie/Workspaces/ros_sonia_ws/src/auv_characterization/Matlab/Data/Test 9 Décembre/Numerical data/y_15_1.mat', 0.13,10,2);

data0      = [x02];
speed0     = filter(b, 1, data0);
thrust0    = zeros(1, length(data0));
thrust0(:) = 29.0884;

x21 = ParseData('/home/olavoie/Workspaces/ros_sonia_ws/src/auv_characterization/Matlab/Data/Test 9 Décembre/Numerical data/y_25_1.mat', 0.23,10,2); 

data2      = [x21];
speed2     = filter(b, 1, data2);
thrust2    = zeros(1, length(data2));
thrust2(:) = 49.3131;

x31 = ParseData('/home/olavoie/Workspaces/ros_sonia_ws/src/auv_characterization/Matlab/Data/Test 9 Décembre/Numerical data/y_30_1.mat', 0.28,10,2);
x32 = ParseData('/home/olavoie/Workspaces/ros_sonia_ws/src/auv_characterization/Matlab/Data/Test 9 Décembre/Numerical data/y_30_2.mat', 0.28,10,2);

data3      = [x31, x32];
speed3     = filter(b, 1, data3);
thrust3    = zeros(1, length(data3));
thrust3(:) = 3.942224025726318;

speed  = [speed1, speed2, speed3, speed0];
thrust = [thrust1, thrust2, thrust3, thrust0];

data = [speed; thrust];
data = conj(data');

vv = data(:, 1) .* data(:, 1);
v  = data(:, 1);

speed = [v, vv];
r = inv(conj(speed') * speed) * conj(speed') * conj(thrust')

function x = ParseData(file_name, threshold1, threshold2, axis) 
 load(file_name);
 LinearSpeed  = LINEAR_SPEED;
 
 windowSize = 5; 
 b = (1/windowSize)*ones(1,windowSize);
 LinearSpeedFilter = filter(b,1, LinearSpeed(:, axis));
 
 t = length(LinearSpeed(:,axis));
 j = 0;
 for i = 1:t
     
     if (LinearSpeedFilter(i) >= threshold1) && (LinearSpeedFilter(i) <= threshold2)
         j = j + 1;
         x(j) = LinearSpeedFilter(i);
     end
     
 end
 hold on 
 plot(1:length(x),x)
end
 
 
 