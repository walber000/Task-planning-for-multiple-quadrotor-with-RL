%% LA define trajetoria/waypoints
%run('LA_trajetoria8.m')
%clear all; close all;

%% ROS
rosshutdown
if(~robotics.ros.internal.Global.isNodeActive)
    Eug = '192.168.1.119';
    rosinit(Eug)
end

%% number of drones
n = 3;

%% PUBLISHER
for i=1:n
    pub_takeoff{i} = rospublisher(['/drone_',num2str(i),'/takeoff'], 'std_msgs/Empty');
    pub_landing{i} = rospublisher(['/drone_',num2str(i),'/land'], 'std_msgs/Empty');
    pub_cmd{i} = rospublisher(['/cmd_vel_',num2str(i)]);
end
pub_set = rospublisher('/gazebo/set_model_state');

%% SUBSCRIBER AND MESSAGES
for i=1:n
    subs_pos{i} = rossubscriber(['/drone_',num2str(i),'/gt_pose']);
    msg_cmd{i} = rosmessage('geometry_msgs/Twist');
end

msg_empty = rosmessage(pub_takeoff{1});
msg_set = rosmessage('gazebo_msgs/ModelState');

%% Set inicial position
inicial_pos = [1 0 0; 2 0 0; 3 0 0];
for i=1:n
    send(pub_landing{i},msg_empty);
    pause(0.1)
    msg_set.ModelName = ['drone_',num2str(i)];
    msg_set.Pose.Position.X = inicial_pos(i,1);
    msg_set.Pose.Position.Y = inicial_pos(i,2);
    msg_set.Pose.Position.Z = inicial_pos(i,3);
    quat = eul2quat([0 0 0]);
    msg_set.Pose.Orientation.W = quat(1);
    msg_set.Pose.Orientation.X = quat(2);
    msg_set.Pose.Orientation.Y = quat(3);
    msg_set.Pose.Orientation.Z = quat(4);
    send(pub_set,msg_set);
    pause(0.1)
end
%% waypoints
h = [2 2 2];
for i=1:n
    for j=1:size(waypoints2(i,:),2)
        if(isempty(waypoints2{i,j}))
            break
        else
            wp(j,:,i) = [waypoints2{i,j}];
            
            %ij to ji to hard-fix problem
%             aux = wp(j,1,i);
%             wp(j,1,i) = wp(j,2,i);
%             wp(j,2,i) = aux;
            x_real = wp(j,2,i); y_real = wp(j,1,i);
            
            wp(j,1,i) = x_real*20/160 - 10;
            wp(j,2,i) = 10 - y_real*20/160;
        end
%         if(wp(j,3,i)~=0) %separates first batch of wp
%             break
%         end
    end
end

%% figure settings
% fig = figure;
% axis([-10 10 -10 10 0 5])
% grid on;
% hold on;
% y = zeros(1,3);
% plot3(wp(:,1,1),wp(:,2,1),ones(length(wp),1),'r*')

%% controller
tol = 0.4;
kp = 0.2;
for i=1:n
    controller{i} = robotics.PurePursuit();
    next_wp{i} = get_new_wp(waypoints2,i,1);
    controller{i}.Waypoints = next_wp{i}(:,1:2);
    controller{i}.DesiredLinearVelocity = 0.25;
    controller{i}.LookaheadDistance = 1;
    controller{i}.MaxAngularVelocity = 0.55;
end
%% takeoff
for i=1:n
    send(pub_takeoff{i},msg_empty);
    pause(0.1);
end
pause(3);

%%
k = 1; %iterador
x = k;
finished = [0 0 0];
batch = [1 1 1];

fig=figure();
for i=1:3
    subplot(1,3,i)
    plot(next_wp{i}(:,1),next_wp{i}(:,2))
    axis([-10 10 -10 10])
    grid on
end

while(k<500000)
    for i=1:n
        if(~finished(i))
            %% sensors
            pos{i} = receive(subs_pos{i},2); %2 = timeout
            xyz(i,:) = [pos{i}.Position.X, pos{i}.Position.Y, pos{i}.Position.Z];
            disp(i+": "+(xyz(i,1)+" "+(xyz(i,2))))
            next_wp{i}(end,1)
            next_wp{i}(end,2)
            quat(i,:) = [pos{i}.Orientation.W, pos{i}.Orientation.X, pos{i}.Orientation.Y, pos{i}.Orientation.Z];
            att(i,:) = quat2eul(quat(i,:), 'XYZ');
%             y(i,:,end+1)=xyz(i,:);
            %% movimentação
            [vx(i), wz(i), soft_wp(i,:)] = controller{i}([xyz(i,1:2) att(i,3)]');
            msg_cmd{i}.Linear.X = vx(i);
            msg_cmd{i}.Angular.Z = wz(i);
            erro(i) = h(i) - xyz(i,3);
            msg_cmd{i}.Linear.Z = kp*erro(i);
            send(pub_cmd{i}, msg_cmd{i});
            pause(0.01)
            %% plot
            %p = plot3(reshape(y(1,1,2:end), [], 1),reshape(y(1,2,2:end), [], 1),reshape(y(1,3,2:end), [], 1),'Color',[0 1 0], 'LineWidth', 2);
    %         if(k>10)
    %             delete(p(i,:,1:end-10));
    %         end
            %plot3(next_wp(1),next_wp(2),1,'k*')
            %axis([-5 5 -5 5 0 5])
            %disp(k)
            k=k+1;
            x(end+1)=k;
            %% if reached last wp, land
            if(abs((xyz(i,1)-next_wp{i}(end,1)))<tol && abs((xyz(i,2)-next_wp{i}(end,2)))<tol)
                batch(i)=batch(i)+1;
                next_wp{i} = get_new_wp(waypoints2,i,batch(i));
                if(isempty(next_wp{i}))
                    disp("reached last wp, landing...")
                    %all velocities to zero
                    msg_cmd{i}.Linear.X = 0;
                    msg_cmd{i}.Angular.Z = 0;
                    send(pub_cmd{i}, msg_cmd{i});
                    pause(0.01)
                    %landing
                    send(pub_landing{i},msg_empty);
                    pause(0.01)
                    finished(i) = finished(i) + 1;
                else
                    disp(i+" getting new batch("+batch(i)+")...")
                    controller{i}.Waypoints = next_wp{i}(:,1:2);
                end
            end
        else
            if(sum(finished)==n)
                break
            end
        end
    end
    if(sum(finished)==n)
        break
    end
    %% exit if figure is closed
    if(~ishandle(fig))
        disp("figure closed")
        break
    end
end

%% END SECTION
disp("end section")
for i = 1:n
    %all velocities to zero
    msg_cmd{i}.Linear.X = 0;
    msg_cmd{i}.Linear.Z = 0;
    msg_cmd{i}.Angular.Z = 0;
    send(pub_cmd{i}, msg_cmd{i});

    %landing
    send(pub_landing{i},msg_empty);
    pause(0.1);
end
%rosshutdown
rosshutdown

function [next_wp] = get_new_wp(waypoints2,n,batch)
    count=1;
    next_wp = [];
    for j=1:length(waypoints2(n,:))
        if(isempty(waypoints2{n,j}))
            break
        elseif(count==batch)
            next_wp(end+1,:) = [waypoints2{n,j}];
            x_real = next_wp(end,2); y_real = next_wp(end,1);            
            next_wp(end,1) = x_real*20/160 - 10;
            next_wp(end,2) = 10 - y_real*20/160;
        end
        if(waypoints2{n,j}(3)~=0) %separates first batch of wp
            count = count + 1;
            if(count>batch)
                break
            end
        end
    end
end
%EOF