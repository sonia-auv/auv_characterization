clear
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
x11 = ParseData('/home/olavoie/Workspaces/ros_sonia_ws/src/auv_characterization/Matlab/Data/Test 9 Décembre/Numerical data/yaw_10_1.mat', 0.379,10,3);

data1      = [x11];
speed1     = filter(b,1, data1);
thrust1    = zeros(1, length(speed1));
thrust1(:) = 1.155118584632874;

x21 = ParseData('/home/olavoie/Workspaces/ros_sonia_ws/src/auv_characterization/Matlab/Data/Test 9 Décembre/Numerical data/yaw_15_1.mat', 0.38,10,3); 

data2      = [x21];
speed2     = filter(b,1, data2);
thrust2    = zeros(1, length(speed2));
thrust2(:) = 1.155118584632874;

speed  = [speed1];
thrust = [thrust1];

data = [speed; thrust];
data = conj(data');

vv = data(:, 1) .* data(:, 1);
v  = data(:, 1);

speed = [v, vv];
r = inv(conj(speed') * speed) * conj(speed') * conj(thrust');

function x = ParseData(file_name, threshold1, threshold2, axis) 
 load(file_name);
 LinearSpeed  = ANGULAR_SPEED;
 
 windowSize = 500; 
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
 
 
 