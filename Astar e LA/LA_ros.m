%% LA define trajetoria/waypoints
%run('LA_trajetoria8.m')

%% ROS publica waypoins em topico /wp
%rosshutdown
if(~robotics.ros.internal.Global.isNodeActive)
    Olympus = '192.168.0.41';
    rosinit(Olympus)
end
%%PUBLISHER
%publisher = rospublisher('/wp', 'std_msgs/Float32MultiArray');
publisher = rospublisher('/wp', 'std_msgs/UInt32MultiArray');
msg = rosmessage(publisher);

%define message layout
dim_array = [rosmessage('std_msgs/MultiArrayDimension') rosmessage('std_msgs/MultiArrayDimension') rosmessage('std_msgs/MultiArrayDimension')];
layout = rosmessage('std_msgs/MultiArrayLayout');
max_wp = max(cellfun('size',waypoints2,1));

layout.Dim = dim_array;
layout.Dim(1).Label  = "traj";
layout.Dim(1).Size   = length(waypoints2);
layout.Dim(1).Stride = length(waypoints2)*max_wp*2;
layout.Dim(2).Label  = "wp";
layout.Dim(2).Size   = max_wp;
layout.Dim(2).Stride = max_wp*2;
layout.Dim(3).Label  = "ij";
layout.Dim(3).Size   = 2;
layout.Dim(3).Stride = 2;

msg.Layout = layout;

% cria mensagem
for i = 1:length(waypoints2)
    for j = 1:max_wp
        if(j<=(cellfun('size',waypoints2(i),1)))
            msg.Data((i-1)*max_wp*2 + (j-1)*2 + 1) = waypoints2{i}(j,1);
            msg.Data((i-1)*max_wp*2 + (j-1)*2 + 2) = waypoints2{i}(j,2);
        else
            msg.Data((i-1)*(max_wp)*2 + (j-1)*2 + 1) = 0;
            msg.Data((i-1)*(max_wp)*2 + (j-1)*2 + 2) = 0;
        end
    end
end

% publica mensagem
send(publisher,msg);
disp(msg)
