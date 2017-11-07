% Read out ROStopics, originated from the argos_bridge package plots it in a polar plot.
% Required: Matlab robotics toolbox (with ROS capabilities)
% Writen by k. Mcguire (k.n.mcguire@tudelft.nl) Date: 28/10/12

clc;
close all;
clear all;


% First start up the rosnode with: rosinit
% When finished: rosshutdown

% If new msgs are made, run the these lines and restart matlab
%  folderpath = '/home/knmcguire/Documents/Software/catkin_ws/src'
%  rosgenmsg(folderpath);

% Subscriber and publishers
rabsub = rossubscriber('/bot0/rangebearing');
cmd_vel_pub = rospublisher('/bot0/cmd_vel', 'geometry_msgs/Twist');

proxsub = rossubscriber('/bot0/proximity');
twist = rosmessage('geometry_msgs/Twist');
rab_data = rosmessage('argos_bridge/RangebearingList');
pause(2)

figure(1),
while(1)
   % receive and send data
   %send(cmd_vel_pub, twist);
   proxdata = receive(proxsub,1);
   % rab_data = receive(rabsub,1);
   % Polar Plot relative positions other footbots  

   disp(proxdata.N)
   disp(proxdata.Proximities)
   for i=1:rab_data.N

      polarplot(rab_data.Rangebearings(i).Angle, rab_data.Rangebearings(i).Range,'o')
      rlim([0 4]) %Range only goes untill 300 cm (3 meter)
      
      hold on,
   end
   hold off
end


