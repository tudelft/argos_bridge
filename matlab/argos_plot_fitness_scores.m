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
fitnesssub = rossubscriber('/fitness_score_array');
pause(2)

figure(1),

mean_fitness = [];
while(1)
   % receive and send data
   %send(cmd_vel_pub, twist);
   fitnessdata = receive(fitnesssub);
   
      mean_fitness = [mean_fitness mean(fitnessdata.Data(2:end))];
      
      plot(fitnessdata.Data(1), fitnessdata.Data(2:end),'kx')
      hold on,
      plot(0:(length(mean_fitness)-1),mean_fitness ,'r-o')

      xlim([0 20]) 
      
      hold on,
end



