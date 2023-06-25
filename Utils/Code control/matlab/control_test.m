IPvm='192.168.153.128';

% rosshutdown;

% rosinit(IPvm);
gzinit(IPvm, 14581);


modelList=gzmodel("list");

% hilite_system('performCoSimulationWithGazebo/Gazebo Pacer')
% rosshutdown;