% old = 1;
% for i = 1:158
%     map2(:,i) = old;
%     if(i~=158)
%         old = map2(:,i+1);
%     end
%     map2(:,i+1) = map2(:,i);
% end

% map2 = cat(2, ones(size(map2, 1), 2), map2);
% map2 = cat(2, map2, ones(size(map2, 1), 3));
% map2 = cat(1, ones(2, size(map2, 2)), map2);
% map2 = cat(1, map2, ones(2, size(map2, 2)));

for i=1:n
    for j=1:size(waypoints2(i,:),2)
        if(isempty(waypoints2{i,j}))
            break
        else
            wp(j,:,i) = [waypoints2{i,j}];
            x_real = wp(j,2,i); y_real = wp(j,1,i);
            wp(j,1,i) = x_real*20/160 - 10;
            wp(j,2,i) = 10 - y_real*20/160;
            [posx(j,1,i), posx(j,2,i)] = pos_to_ij(wp(j,1:2,i));
        end
%         if(wp(j,3,i)~=0) %separates first batch of wp
%             break
%         end
    end
end

function [i, j] = pos_to_ij(pos)
    num_lin = 160; num_col = 160;
    pos(1) = pos(1) + 10; %offset, pois (0,0) estava no centro do mapa
    pos(2) = 10 - pos(2); %offset
    i = round(pos(2)*num_lin/20);
    j = round(pos(1)*num_col/20);
end