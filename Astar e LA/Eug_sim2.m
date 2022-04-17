%% LA define trajetoria/waypoints
%run('LA_trajetoria8.m')
%clear all; close all;

%% ROS
rosshutdown

%hardcodar teleporte inicial dos obj
%pequeno pause na hora do loading, antes de subir


% [i, j] = pos_to_ij([2 7])
% x_real = i; y_real = j;
% next_(1) = x_real*20/160 - 10;
% next_(2) = 10 - y_real*20/160;
% next_
% return

if(~robotics.ros.internal.Global.isNodeActive)
    Eug = '192.168.1.119';
    rosinit(Eug)
end

%% number of drones
n = 4;

%% PUBLISHER
for i=1:n
    pub_takeoff{i} = rospublisher(['/drone_',num2str(i),'/takeoff'], 'std_msgs/Empty');
    pub_landing{i} = rospublisher(['/drone_',num2str(i),'/land'], 'std_msgs/Empty');
    pub_cmd{i} = rospublisher(['/cmd_vel_',num2str(i)]);
end
for i=1:n
    pub_takeoff_l{i} = rospublisher(['/drone_',num2str(i),'_load/takeoff'], 'std_msgs/Empty');
    pub_landing_l{i} = rospublisher(['/drone_',num2str(i),'_load/land'], 'std_msgs/Empty');
    pub_cmd_l{i} = rospublisher(['/cmd_vel_',num2str(i),'_load']);
end
pub_set = rospublisher('/gazebo/set_model_state');

%% SUBSCRIBER AND MESSAGES
for i=1:n
    subs_pos{i} = rossubscriber(['/drone_',num2str(i),'/gt_pose']);
    subs_pos_l{i} = rossubscriber(['/drone_',num2str(i),'_load/gt_pose']);
    msg_cmd{i} = rosmessage('geometry_msgs/Twist');
end

msg_empty = rosmessage(pub_takeoff{1});
msg_set = rosmessage('gazebo_msgs/ModelState');

%% Set inicial position
%inicial_pos = [1 0 0; 2 0 0; 3 0 0];
%inicial_pos = [1 0 0; 1 -6 0; 3 0 0];
%inicial_pos = [1 0 0; -5 2 0; 3 0 0];
inicial_pos = [0 8 0.1; -8 0 0.1; 8 0 0.1; 0 -8 0.1];
for i=1:n
    send(pub_landing{i},msg_empty);
    pause(0.1)
    msg_set.ModelName = ['drone_',num2str(i)];
    msg_set.Pose.Position.X = inicial_pos(i,1);
    msg_set.Pose.Position.Y = inicial_pos(i,2);
    msg_set.Pose.Position.Z = inicial_pos(i,3);
    quat = eul2quat([-pi+(pi*i)/2 0 0]);
    if(i==3)
        quat = eul2quat([pi 0 0]);
    end
    if(i==4)
        quat = eul2quat([pi/2 0 0]);
    end
    msg_set.Pose.Orientation.W = quat(1);
    msg_set.Pose.Orientation.X = quat(2);
    msg_set.Pose.Orientation.Y = quat(3);
    msg_set.Pose.Orientation.Z = quat(4);
    send(pub_set,msg_set);
    pause(0.1)
end

%% inicial pos for load drones
inicial_pos_l = [50 50 0; 49 50 0; 48 50 0; 47 50 0];
for i=1:n
    send(pub_landing_l{i},msg_empty);
    pause(0.1)
    msg_set.ModelName = ['drone_',num2str(i),'_load'];
    msg_set.Pose.Position.X = inicial_pos_l(i,1);
    msg_set.Pose.Position.Y = inicial_pos_l(i,2);
    msg_set.Pose.Position.Z = inicial_pos_l(i,3);
    quat = eul2quat([0 0 0]);
    msg_set.Pose.Orientation.W = quat(1);
    msg_set.Pose.Orientation.X = quat(2);
    msg_set.Pose.Orientation.Y = quat(3);
    msg_set.Pose.Orientation.Z = quat(4);
    send(pub_set,msg_set);
    pause(0.1)
end

load_names = {'walber_load_1','walber_load_2','walber_load_3','walber_load_4','walber_load_5'};
unload_names = {'walber_unload_1','walber_unload_2','walber_unload_3','walber_unload_4'};
%load_pos = [-8 8 0.5;8 8 0.5;0 -5 0.5];
load_pos = [0 0 0.3;0.15 0.15 0;-0.15 0.15 0; 0.15 -0.15 0; -0.15 -0.15 0];
unload_pos = [0 8 0; -8 0 0; 8 0 0; 0 -8 0];
for i=1:length(load_names)
    msg_set.ModelName = ['walber_load_',num2str(i)];
    msg_set.Pose.Position.X = load_pos(i,1);
    msg_set.Pose.Position.Y = load_pos(i,2);
    msg_set.Pose.Position.Z = load_pos(i,3);
    send(pub_set,msg_set);
    pause(0.1)
end

%% waypoints
h = 2*ones(1,n);
clear wp
for i=1:n
    for j=1:size(waypoints2(i,:),2)
        if(isempty(waypoints2{i,j}))
            break
        else
            wp(j,:,i) = [waypoints2{i,j}];
            x_real = wp(j,2,i); y_real = wp(j,1,i);
            wp(j,1,i) = x_real*20/160 - 10;
            wp(j,2,i) = 10 - y_real*20/160;
        end
%         if(wp(j,3,i)~=0) %separates first batch of wp
%             break
%         end
    end
end

wp2 = [];
%% Get just waypoints of load/unload/end
for i=1:n
    aux = 0;
    for j=1:size(wp,1)
        if(wp(j,3,i)~=0)
            aux = aux + 1;
            wp2(aux,:,i) = wp(j,:,i);
        end
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
tol_ang = pi/30; %6 graus
vel_ang = 0.55;
vel_lin = 0.25;
kp = 0.2;
clear next_wp
for i=1:n
    controller{i} = robotics.PurePursuit();
    next_wp(i,:) = wp(1,:,i);
%     if(i==2)
%         next_wp(i,:) = wp(1,:,i);
%     end
    %next_wp{i} = get_new_wp(waypoints2,i,1);
    controller{i}.Waypoints = next_wp(i,1:2);
    controller{i}.DesiredLinearVelocity = vel_lin;
    controller{i}.LookaheadDistance = 1;
    controller{i}.MaxAngularVelocity = vel_ang;
    disp(i+" getting new wp: "+next_wp(i,1)+" "+next_wp(i,2))
end
%% takeoff
for i=1:n
    send(pub_takeoff{i},msg_empty);
    pause(0.15);
end
pause(0.1)
for i=1:n
    send(pub_takeoff_l{i},msg_empty);
    pause(0.15);
end
pause(4);

%%
k = 1; %iterador
x = k;
finished = zeros(1,n);
batch = ones(1,n);
wp_counter = ones(1,n);
conflict = zeros(1,n);
astar = zeros(1,n);
loading = zeros(1,n);
loaded = zeros(1,n);
time_landing = zeros(1,n);
num_lin = size(map,1);
num_col = size(map,2);
conflict_dist = 2.2;
priority = 1:n;
raio = 0.875;
raio_mult = 1.1;
raio_grid = 7;
end_conflict_k = 0;
end_conflict_mult = 1;
end_conflict_range = 7; %number of iters with reduced conflict_dist
load_iter_k = zeros(1,n);
load_time = 10;

fig=figure();
% for i=1:3
%     subplot(1,3,i)
%     plot(next_wp(i,1),next_wp(i,2),'b*')
    axis([-10 10 -10 10])
    grid on
    hold on
% end

for i=1:n  
    pos{i} = receive(subs_pos{i},2); %2 = timeout
    xyz(i,:) = [pos{i}.Position.X, pos{i}.Position.Y, pos{i}.Position.Z];
    pause(0.02)
end

while(k<500000)
    for i=1:n
        if(~finished(i))
            %% sensors
            if(~loaded(i))
                pos{i} = receive(subs_pos{i},2); %2 = timeout
            else
                pos{i} = receive(subs_pos_l{i},2);
            end
            xyz(i,:) = [pos{i}.Position.X, pos{i}.Position.Y, pos{i}.Position.Z];         
            quat(i,:) = [pos{i}.Orientation.W, pos{i}.Orientation.X, pos{i}.Orientation.Y, pos{i}.Orientation.Z];
            att(i,:) = quat2eul(quat(i,:), 'XYZ');

            %% calculate distance between drones
            aux2=0;
            for j=1:n
                if i~=j
                    dist_matrix(i,j) = sqrt((xyz(i,1)-xyz(j,1))^2 + (xyz(i,2)-xyz(j,2))^2);
                    if(dist_matrix(i,j)<conflict_dist)
                        if(conflict(i)~=0)
                            if(priority(j)>priority(conflict(i)))
%                                 disp("here")
                                disp(i+" conflict from "+conflict(i)+" to "+j)
                                conflict(i) = j; %fica em conflito com drone de maior prioridade
                                astar(i) = 0;
                            end
                        else
                            conflict(i) = j;
                        end
                        aux2=1;
                    else
                        if(conflict(i)==j) %se conflito com akele drone acabou
                            conflict(i)=0;
                        end
                    end
                end
            end
            if(aux2==0)
                conflict(i)=0;
%                 disp(i+" end conflict")
            end
            %% movimentação
%             disp(i+" conflict: "+conflict(i))
%             disp(i+" astar: "+astar(i))
            if(~conflict(i) && (astar(i) == 0))
                controller{i}.DesiredLinearVelocity = vel_lin;
                controller{i}.MaxAngularVelocity = vel_ang;
                [vx(i), wz(i), soft_wp(i,:)] = controller{i}([xyz(i,1:2) att(i,3)]');
            elseif(astar(i) || (priority(i) > priority(conflict(i))))     
                % in conflict, generate wp's with Astar and moves using
                % pitch and yaw separately
                if(~astar(i))
                    skip_astar(i) = 0;
                    aux(i) = 0;
                    obs_count = 0;
                    obs = [0 0];
                    for j = 1:n
                        if(j~=i)
                            obs_count = obs_count + 1;
                            [i1(j), j1(j)] = pos_to_ij(xyz(j,1:2));
                            obs(obs_count,:) = [i1(j) j1(j)];
                        end
                    end
                    %plot(wp(wp_counter(i)-3:wp_counter(i)+5,1:2,i)-10,wp(wp_counter(i)-3:wp_counter(i)+5,1:2,i),'k*');
                    [i1(i), j1(i)] = pos_to_ij(xyz(i,1:2));
                    
                    if(sum(wp(wp_counter(i):min(size(wp,1),wp_counter(i)+5),3,i))~=0)
                        for a=1:6
                            d(a) = norm(wp(max(wp_counter(i)+a-1,1),1:2,i) - xyz(conflict(i), 1:2));
                        end 
                        if(sum(d<raio*raio_mult))
                            disp("load/unload blocked, changing priority...");
%                             wp_counter(i) = wp_counter(i) - aux(i);
                            aux_priority = priority(i);
                            priority(i) = priority(conflict(i));
                            priority(conflict(i)) = aux_priority;
                            priority
                            skip_astar(i) = 1;
                            break
                        end
                    end
                    
                    while(norm(wp(wp_counter(i),1:2,i) - xyz(conflict(i), 1:2)) < raio*raio_mult)                        
                        disp("skipíng wp...")

                        if(sum(wp(wp_counter(i):min(size(wp,1),wp_counter(i)+5),3,i))~=0)
                            for a=1:6
                                d(a) = norm(wp(max(wp_counter(i)+a-1,1),1:2,i) - xyz(conflict(i), 1:2));
                            end 
                            if(sum(d<raio*raio_mult))
                                disp("load/unload blocked2, changing priority...");
                                wp_counter(i) = wp_counter(i) - aux(i);
                                aux_priority = priority(i);
                                priority(i) = priority(conflict(i));
                                priority(conflict(i)) = aux_priority;
                                priority
                                skip_astar(i) = 1;
                                break
                            end
                        end
                        
                        wp_counter(i) = wp_counter(i) + 1;
                        aux(i) = aux(i) + 1;
                    end
                    if(skip_astar(i))
                        vx(i) = 0;
                        wz(i) = 0;
                        disp(i+" stopped to skip")
                    else
                        [i2(i), j2(i)] = pos_to_ij(wp(wp_counter(i),1:2,i));
                        next_wp_Astar = [];
                        [~, ~, ~, ~, next_wp_Astar] = Astar_multi_sim(map,[i1(i) j1(i)], [i2(i) j2(i)], 0, 0, 1, obs, raio_grid);
                        disp(i+" new astar")
                        astar(i)=1;
%                         disp("astar "+i)
                        size_astar(i) = size(next_wp_Astar,1);
                        for a=1:size_astar(i)
                            next_wp_astar_cell{i,a}= [next_wp_Astar(a,:)];

                            x_real = next_wp_astar_cell{i,a}(1); y_real = next_wp_astar_cell{i,a}(2);
                            next_wp(i,1) = x_real*20/160 - 10;
                            next_wp(i,2) = 10 - y_real*20/160;
                            next_wp(i,3) = 0;
                            plot(next_wp(i,1),next_wp(i,2),'k*');
                        end
                        z(i) = 1;
                        aux_plot = 0;
                    end
                end
                if(skip_astar(i))
                    skip_astar(i) = 0;
                else
                    [i1(i), j1(i)] = pos_to_ij(xyz(i,1:2));
                    x_real = next_wp_astar_cell{i,z(i)}(1); y_real = next_wp_astar_cell{i,z(i)}(2);
                    next_wp(i,1) = x_real*20/160 - 10;
                    next_wp(i,2) = 10 - y_real*20/160;
                    next_wp(i,3) = 0;

                    if(aux_plot==0)
                        plot(next_wp(i,1),next_wp(i,2),'b*');
                        aux_plot = 1;
                    end
                    [i2(i), j2(i)] = pos_to_ij(next_wp(i,:));
                    ang(i) = atan2(-(j2(i)-j1(i)), (i2(i)-i1(i))); %i-axis invertido
                    if(i1(i)==i2(i) && j1(i)==j2(i))
                        erro_ang(i) = 0;
                    else
                        erro_ang(i) = ang(i) - att(i,3);
                        disp("desejado: "+ang(i)*180/pi+", atual: "+att(i,3)*180/pi)
                    end
                    if(abs(erro_ang(i))>tol_ang)
                        mod_vel = 0.33 + 0.22*(abs(erro_ang(i))>pi/10);
                        %disp("yaw, erro_ang = "+erro_ang(i))
                        if(abs(erro_ang(i))>pi)
                            wz(i) = -sign(erro_ang(i))*vel_ang*mod_vel;
                        else
                            wz(i) = sign(erro_ang(i))*vel_ang*mod_vel;
                        end 
                        vx(i) = 0;
                    else
                        if(pdist([xyz(i,1:2); next_wp(i,1:2)]) > tol)
                            vx(i) = vel_lin*0.7;
%                             disp("linear")
                            wz(i) = 0;
                        else
                            z(i) = z(i) + 1;
                            aux_plot = 0;
                            disp(i+ " next astar wp")
                            if(z(i)>size_astar(i))
                                astar(i) = 0;
                                disp(i+ " done astar")
                                conflict_dist = conflict_dist*end_conflict_mult;
                                end_conflict_k = k;
                                next_wp(i,3) = wp(wp_counter(i),3,i);
                            end
                        end
                    end
                end
            else % em conflito e prioridade menor
                vx(i) = 0;
                wz(i) = 0;
%                 disp(i+" stopped")
            end
            %disp(i+" vx: "+vx(i)+", wz: "+wz(i))
            if(loading(i)==0 || loading(i)==3)
                msg_cmd{i}.Linear.X = vx(i);
                msg_cmd{i}.Angular.Z = wz(i);
                erro(i) = h(i) - xyz(i,3);
                msg_cmd{i}.Linear.Z = kp*erro(i);
            else
                msg_cmd{i}.Linear.X = 0;
                msg_cmd{i}.Angular.Z = 0;
                erro(i) = h(i) - xyz(i,3);
                msg_cmd{i}.Linear.Z = (0.6-0.35*(abs(erro(i))<0.25))*sign(erro(i));
            end
            if(loaded(i))
                send(pub_cmd_l{i}, msg_cmd{i});
            else
                send(pub_cmd{i}, msg_cmd{i});  
            end
            pause(0.1)
            %% plot
            %p = plot3(reshape(y(1,1,2:end), [], 1),reshape(y(1,2,2:end), [], 1),reshape(y(1,3,2:end), [], 1),'Color',[0 1 0], 'LineWidth', 2);
    %         if(k>10)
    %             delete(p(i,:,1:end-10));
    %         end
            %plot3(next_wp(1),next_wp(2),1,'k*')
            %axis([-5 5 -5 5 0 5])
            %disp(k)
            
%             subplot(1,3,i)
            %plot(next_wp(i,1),next_wp(i,2),'b*')
%             if(mod(k,10)==0)
%                 plot(xyz(1,1),xyz(1,2),'b*','MarkerSize',2);
%                 plot(xyz(2,1),xyz(2,2),'r*','MarkerSize',2);
%                 plot(xyz(3,1),xyz(3,2),'g*','MarkerSize',2);
%                 plot(xyz(4,1),xyz(4,2),'k*','MarkerSize',2);
%             end
            %% get new waypoint if reached the current one
            if(abs((xyz(i,1)-next_wp(i,1)))<tol && abs((xyz(i,2)-next_wp(i,2)))<tol)
                % if reached last wp, land
                if(next_wp(i,3)==3)
                    disp(i+ " reached last wp, landing...")
                    msg_cmd{i}.Linear.X = 0;
                    msg_cmd{i}.Angular.Z = 0;
                    msg_cmd{i}.Linear.Z = -0.5;
                    send(pub_cmd{i}, msg_cmd{i});
                    pause(0.1)
                    time_landing(i) = now;               
                    finished(i) = finished(i) + 1;
                    xyz(i,1:2) = [-99 -99]; %change "position" to avoid conflicts
                else
                    if(wp(wp_counter(i),3,i)~=0)
                        if(wp(wp_counter(i),3,i)==1)
                            if(loading(i)==0)
                                disp(i+" loading...")
                            end
                            if(loading(i)<=1)
                                h(i) = 1;
                                erro(i) = h(i) - xyz(i,3);
                                loading(i) = 1;
                                if(abs(erro(i))<0.3)
                                    if(load_iter_k(i)==0)
                                        load_iter_k(i) = k;
                                    end                                   
                                    if(k-load_iter_k(i)>load_time)
                                        loading(i) = 2;
                                        disp(i+" loaded, going up...")
                                        loaded(i) = 1;

                                        msg_set.ModelName = ['drone_',num2str(i)];
                                        msg_set.Pose.Position.X = inicial_pos_l(i,1)+5;
                                        msg_set.Pose.Position.Y = inicial_pos_l(i,2)+5;
                                        msg_set.Pose.Position.Z = inicial_pos_l(i,3)+2;
                                        send(pub_set,msg_set);
                                        pause(0.1)
                                        msg_cmd{i}.Linear.X = 0;
                                        msg_cmd{i}.Angular.Z = 0;
                                        msg_cmd{i}.Linear.Z = 0;
                                        send(pub_cmd{i}, msg_cmd{i}); %changed here
                                        pause(0.1)
                                        
                                        msg_set.ModelName = ['drone_',num2str(i),'_load'];
                                        msg_set.Pose.Position.X = xyz(i,1);
                                        msg_set.Pose.Position.Y = xyz(i,2);
                                        msg_set.Pose.Position.Z = xyz(i,3);
                                        msg_set.Pose.Orientation.W = quat(i,1);
                                        msg_set.Pose.Orientation.X = quat(i,2);
                                        msg_set.Pose.Orientation.Y = quat(i,3);
                                        msg_set.Pose.Orientation.Z = quat(i,4);
                                        send(pub_set,msg_set);
                                        pause(0.1)
                                        
                                        iknn_l(i) = knnsearch(load_pos(:,1:2), xyz(i,1:2));
                                        msg_set.ModelName = load_names{iknn_l(i)};
                                        msg_set.Pose.Position.X = -49;
                                        msg_set.Pose.Position.Y = -49;
                                        load_pos(iknn_l(i),1:2) = [-49 -49];
                                        msg_set.Pose.Position.Z = 1;
                                        msg_set.Pose.Orientation.W = 1;
                                        msg_set.Pose.Orientation.X = 0;
                                        msg_set.Pose.Orientation.Y = 0;
                                        msg_set.Pose.Orientation.Z = 0;
                                        send(pub_set, msg_set);
                                        pause(0.1)
                                        load_iter_k(i) = 0;
                                    end
                                end
                            end
                            if(loading(i)==2)
                                h(i) = 2;
                                erro(i) = h(i) - xyz(i,3);
                                if(abs(erro)<0.3)
                                    disp(i+" done loading")
                                    loading(i) = 3;
                                end
                            end 
                        end
                        if(wp(wp_counter(i),3,i)==2)
                            disp("unload")
                            iknn_un(i) = knnsearch(unload_pos(:,1:2), xyz(i,1:2));
                            msg_set.ModelName = load_names{iknn_l(i)};
                            msg_set.Pose.Position.X = unload_pos(iknn_un(i), 1);
                            msg_set.Pose.Position.Y = unload_pos(iknn_un(i), 2);
                            msg_set.Pose.Position.Z = 1;
                            msg_set.Pose.Orientation.W = 1;
                            msg_set.Pose.Orientation.X = 0;
                            msg_set.Pose.Orientation.Y = 0;
                            msg_set.Pose.Orientation.Z = 0;
                            send(pub_set, msg_set);
                            pause(0.1)
                            
                            msg_set.ModelName = ['drone_',num2str(i),'_load'];
                            msg_set.Pose.Position.X = inicial_pos_l(i,1)+5;
                            msg_set.Pose.Position.Y = inicial_pos_l(i,2)+5;
                            msg_set.Pose.Position.Z = inicial_pos_l(i,3)+2;
                            send(pub_set,msg_set);
                            pause(0.1)
                            msg_cmd{i}.Linear.X = 0;
                            msg_cmd{i}.Angular.Z = 0;
                            msg_cmd{i}.Linear.Z = 0;
                            send(pub_cmd_l{i}, msg_cmd{i});
                            pause(0.1)

                            msg_set.ModelName = ['drone_',num2str(i)];
                            msg_set.Pose.Position.X = xyz(i,1);
                            msg_set.Pose.Position.Y = xyz(i,2);
                            msg_set.Pose.Position.Z = xyz(i,3);
                            msg_set.Pose.Orientation.W = quat(i,1);
                            msg_set.Pose.Orientation.X = quat(i,2);
                            msg_set.Pose.Orientation.Y = quat(i,3);
                            msg_set.Pose.Orientation.Z = quat(i,4);
                            send(pub_set,msg_set);
                            pause(0.1)
                            
                            loaded(i) = 0;
                        end                    
%                         conflict(i) = 0; %stop using Astar if reached load/unload wp
                        batch(i) = batch(i) + 1; %new batch of waypoints, used in Astar
                    end
                    if((loading(i)==0 || loading(i)==3) && astar(i)==0)
                        wp_counter(i) = wp_counter(i) + 1;     
                        next_wp(i,:) = wp(wp_counter(i),:,i);
                        disp(i+" getting new wp("+wp_counter(i)+"): "+next_wp(i,1)+" "+next_wp(i,2))
                        controller{i}.Waypoints = next_wp(i,1:2);
                        loading(i) = 0;
                    end
                end
            end
        else
            if(finished(i)==1)
                %landing after 3s
                time = (now-time_landing(i))*24*60*60;
                if(time>3)
                    msg_cmd{i}.Linear.Z = 0;
                    send(pub_cmd{i}, msg_cmd{i});
                    pause(0.1)
                    disp(i+" landed")
                    send(pub_landing{i},msg_empty);
                    pause(0.1)
                    finished(i) = finished(i) + 1;
                end
            end
            if(sum(finished)==(2*n))
                break
            end
        end        
    end
    k=k+1;
%     if(end_conflict_k>0 && (k-end_conflict_k)>end_conflict_range)
%         conflict_dist = conflict_dist/end_conflict_mult;
%         end_conflict_k = 0;
%     end
    x(end+1)=k;
    if(sum(finished)==(2*n))
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
    pause(0.1)
    send(pub_cmd_l{i}, msg_cmd{i});
    pause(0.1)
    %landing
    send(pub_landing{i},msg_empty);
    pause(0.1)
    send(pub_landing_l{i},msg_empty);
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

function [i, j] = pos_to_ij(pos) %actually, 
    num_lin = 160; num_col = 160;
    pos(1) = pos(1) + 10; %offset, pois (0,0) estava no centro do mapa
    pos(2) = 10 - pos(2); %offset
    i = round(pos(1)*num_lin/20);
    j = round(pos(2)*num_col/20);
end


%EOF