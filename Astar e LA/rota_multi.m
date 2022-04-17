% Learning automata para decisão de melhor sequência de captura de objetos
% em diferentes posições.
% Adaptado de rl_lab.m (LA para soluçao de labirinto) do prof. Cairo.
% Walber Lima P. Junior, 05/09/19

function [dist, waypoints2] = rota_multi(prob1, prob2, p_carga, obj_qtd, demanda_zona)
global num_col num_carga mov_carga mov_descarga debug carga_iter num_obj_decididos
global k3 no_final k2 obj finished no_inic obj_qtd_temp obj2 mov2 num_obj2

%clear all
%close all;
%rand('state',0);

desenhar = 1; pausar = 1; tempo_pause = 0.02; desenhar_demanda = 1;
desenhar_hist = 1; desenhar_3d = 0; debug = 1; conflito = 0;
carga_descarga = 1;

if(desenhar)
    plotMap;
    if(desenhar_3d)
        mapa3 = figure(mapa.Number+1);
        %mapa3 = subplot(2,3,[3 6]);
        h_transp = 50;
        lenX = 290; lenY = 170; grid = 10;
        X = 0:grid:lenX;
        Y = 0:grid:lenY;
        Z = ones(length(Y),length(X));
        mesh(X,Y,Z,'edgecolor',[0.4 0.4 0.4]);
        set(gca,'Ydir','reverse');
        xlim([0 lenX]); ylim([0 lenY]); zlim([0 h_transp+10])
        %mapa3.DataAspectRatio = [1 1 0.4];
        hold on
    end
end
waypoints = [];
waypoints2 = {};
%cor = [1 3];
file = fopen('activemap_multi.txt','r');
size = fscanf(file,'%d',[1 2]);
num_lin = size(1,1);
num_col = size(1,2);
map = fscanf(file,'%d',[num_col num_lin]);
map = map';
num_obj = 0; %iterador para numero de objetos
num_zonas = 0; %iterador para numero de zonas de descarga
prob_conv = 0.98;
prob_conv_carga = 0.7;

num_transp = length(prob1(1,1,:));
count_transp = 0;

for i = 1:num_lin
    for j = 1:num_col
        % if map(i,j) == 0 -> espaço vazio
        % if map(i,j) == 1 -> obstaculo
        if map(i,j) == 2  %Nó inicial no mapa
            count_transp = count_transp + 1;
            i_inic(count_transp) = i;
            j_inic(count_transp) = j;
        end
        if map(i,j) == 3  %Objeto no mapa
            num_obj = num_obj + 1; obj(num_obj,:) = num_col*(i-1)+j;
            i_obj(num_obj)=i; j_obj(num_obj)=j;
        end
        if map(i,j) == 4  %Ponto de entrega no mapa
            num_zonas = num_zonas + 1; zona(num_zonas,1) = i; zona(num_zonas,2) = j;
        end
    end
end

no_inic = num_col*(i_inic-1)+j_inic;

for i = 1:length(no_inic)
%     h_transp = 50;
    if(desenhar_3d)
        %axes(mapa3);
        figure(mapa3.Number)
        pos_img3(i,1)=plot3(j_inic(i)*grid,i_inic(i)*grid,h_transp,'o','Color',[i==3 (i==2)/2 i==1]);
    end
end
if(desenhar_3d)
    %axes(mapa);
    figure(mapa.Number)
end
for i=1:num_zonas
    no_final(i) = num_col*(zona(i,1)-1)+zona(i,2);
end

%Ncel = num_col * num_lin;
num_obj2 = sum(obj_qtd);
font_size = 8;
%colocar quantidade de material em cada zona de carga
for i=1:num_obj
    texto=sprintf('%d',obj_qtd(i));
    obj_cell{i}=text(j_obj(i)+0.3,i_obj(i)-0.3,texto,'HorizontalAlignment','center','Color','black','FontSize',font_size);  
end

carga_zona = zeros(1,num_zonas);

%colocar quantidade de material em cada zona de descarga
for i=1:num_zonas
    texto=sprintf('%d',carga_zona(i));
    zona_cell{i}=text(zona(i,2)+0.3-0.1*desenhar_demanda,zona(i,1)-0.3,texto,'HorizontalAlignment','center','Color','black','FontSize',font_size);  
end

%colocar demanda em cada zona de descarga
if(desenhar_demanda == 1)
    for i=1:num_zonas
        texto=sprintf('/%d',demanda_zona(i));
        text(zona(i,2)+0.5,zona(i,1)-0.3,texto,'HorizontalAlignment','center','Color','black','FontSize',font_size);  
    end
end

%movimentos de carga
for n = 1:num_transp
    for i = 1:num_obj2
        if(max(prob1(i,:,n))<prob_conv)
            mov_carga(n,i) = 0;
        else
            mov_carga(n,i) = find(prob1(i,:,n)==max(prob1(i,:,n)));
        end
    end
end

%movimentos de descarga
for n = 1:num_transp
    for i = 1:num_obj2
        if(max(prob2(i,:,n))<prob_conv)
            mov_descarga(n,i) = 0;
        else
            mov_descarga(n,i) = find(prob2(i,:,n)==max(prob2(i,:,n)));
        end
    end
end

%quantidade de carga
num_carga = 1;
for n = 1:num_transp
    for i = 1:num_obj2-1
        if(max(p_carga(i,:,n))<=prob_conv_carga)
            num_carga(n,i) = 0;
        else
            num_carga(n,i) = min(find(p_carga(i,:,n)==max(p_carga(i,:,n)))); %min evita probs iguais
        end
    end
end
%carga = max(num_carga);

mov_carga
mov_descarga
num_carga

% texto_vel_zero = "vel"+n+":";
% texto_vel_max = "vel"+n+": >>>>>";
dx = 0.1;
ddx = 0.05;
dy = 0.95;
ddy = 0;
barx = 0.05;

for n = 1:num_transp
    
    cor(n) = 2*n-1; %cores impares para carga
%     color(n) = rem(cor,6);
%     R = (color==5||color==0)*(0.5*(rem(color,2))+0.5);
%     G = (color==3||color==4)*(0.5*(rem(color,2))+0.5);
%     B = (color==1||color==2)*(0.5*(rem(color,2))+0.5);
%     color = [R G B];
    pos1(n) = no_inic(n);
    [i1(n),j1(n)]=pos_to_ij(pos1(n));
    waypoints2(n,1) = {[i1(n) j1(n)]};
    %wp_carga(n) = [];
    dist(n) = 0;
    k2(n) = 1; %iterador de ações de carga
    k3(n) = 1; %iterador de ações de descarga
    carga_iter(n) = 0;
    teta(n) = 0;
    finished(n) = 0;
    h(n) = 50;
    loading(n) = 0;
    unloading(n) = 0;
    speed(n) = annotation('textbox', [dx+ddx*n, dy+ddy*n, barx, 0.03], 'string', "vel"+n+":");
end

count_descarga = 0;
tc = 0.4;
obj_qtd_temp = obj_qtd;
num_obj2 = sum(obj_qtd);
obj_sobra = max(0,num_obj2-sum(demanda_zona));
num_obj_decididos = 0;

pause;

%% Decisão de quantos objetos serão capturados por vez 
for n = 1:num_transp
    %max_carga(n)=num_carga(n,1);
    max_carga(n)=decidir_qtd_carga(n);
end
%% Decisão de quais objetos serão capturados
for n = 1:num_transp
    pos2(n)=decidir_carga(n);
    em_carga(n)=1;
end
w=1;
aux_w = zeros(1,3);
waypoints_prev = zeros(n,2);
%% Movimentação
while(count_descarga < num_obj2-obj_sobra || sum(finished)<2*num_transp)
    for n = 1:num_transp
        if(finished(n)<2)
            if(pos1(n)==pos2(n)) %transportador chegou ao destino
                if(finished(n)==1)
                    if(pos1(n)==no_inic(n))
                        if(debug) disp("transp "+n+" terminou e chegou no inicio");end
                        finished(n)=2;
                    end
                elseif((carga_iter(n) < max_carga(n)) && em_carga(n)==1)
                    if(debug&&loading(n)==0) disp(n+" chegou para carregar"); end
                    if((~carga_descarga)||(carga_descarga && loading(n)==4))
                        carga_iter(n) = carga_iter(n) + 1;
                        obj_qtd_temp(mov_carga(n,k2(n)-1))=obj_qtd_temp(mov_carga(n,k2(n)-1))-1;
                        %atualizar quantidade de objetos na zona de carga
                        [i,j]=pos_to_ij(pos1(n));
                        texto=sprintf('%d',obj_qtd_temp(mov_carga(n,k2(n)-1)));
                        delete(obj_cell{mov_carga(n,k2(n)-1)})
                        obj_cell{mov_carga(n,k2(n)-1)}=text(j+0.3,i-0.3,texto,'HorizontalAlignment','center','Color','black','FontSize',font_size);  
                        % marcar X em zona de carga que acabou estoque
                        if(obj_qtd_temp(mov_carga(n,k2(n)-1))==0)
                            text(j,i-0.025,"X",'HorizontalAlignment','center','FontSize',font_size+5,'color','red')
                        end
                        if(carga_iter(n)==max_carga(n))
                            pos2(n) = decidir_descarga(n);
                            em_carga(n) = 0;
                            cor(n) = 2*n;
                        else
                            pos2(n) = decidir_carga(n);
                        end
                        loading(n) = loading(n) + 1;
                    end
                    if(carga_descarga&&loading(n)~=4)
                        if(loading(n)<=1)
                            if(debug) disp(n+" descendo..."); end
                            loading(n) = loading(n) + 1;
                            h(n)=h(n)-10;
                        elseif(loading(n)<=3)
                            if(debug) disp(n+" subindo..."); end
                            loading(n) = loading(n) + 1;
                            h(n)=h(n)+10;
                        elseif(loading(n)==5)
                            loading(n) = 0;
                        end
                    end
                else
                    if(debug&&(unloading(n)==0))  disp(n+" chegou para descarregar"); end
                    if((~carga_descarga) || (carga_descarga && unloading(n)==1))
                        count_descarga = count_descarga + 1;
                        [i,j]=pos_to_ij(pos1(n));
                        %atualizar quantidade de objetos na zona de descarga
                        %carga_zona(mov_descarga(n,obj2(n,k3(n)-1))) = carga_zona(mov_descarga(n,obj2(n,k3(n)-1))) + carga_iter(n); 
                        carga_zona(mov2(n,k3(n)-1)) = carga_zona(mov2(n,k3(n)-1)) + 1; 
                        texto=sprintf('%d',carga_zona(mov2(n,k3(n)-1)));
                        delete(zona_cell{mov2(n,k3(n)-1)})
                        zona_cell{mov2(n,k3(n)-1)}=text(j+0.3-0.1*desenhar_demanda,i-0.3,texto,'HorizontalAlignment','center','Color','black','FontSize',font_size);  
                        carga_iter(n) = carga_iter(n) - 1;
                        if(carga_iter(n)>0)
                            pos2(n) = decidir_descarga(n);
                        elseif(count_descarga ~= num_obj2-obj_sobra)
                            cor(n) = 2*n-1;
                            max_carga(n)=decidir_qtd_carga(n);
                            pos2(n) = decidir_carga(n);
                            em_carga(n) = 1;
                        else
                            pos2(n) = no_inic(n);
                            finished(n)=1;
                            if(debug) disp("transp "+n+" terminou, indo para no_inic"); end
                        end
                        unloading(n) = 0;
                    else
                        if(debug) disp(n+" descarregando..."); end
                        unloading(n) = unloading(n) + 1;
                    end
                end
            end

            [i1(n),j1(n)]=pos_to_ij(pos1(n));
            [i2(n),j2(n)]=pos_to_ij(pos2(n));

            if(loading(n)~=0)
                gsuc(n) = 10; dist(n) = dist(n) + gsuc(n);
                count(n) = 0; waypoints(n,:) = [i1(n) j1(n)];
            else
                if((pos1(n)==pos2(n)))
                    if(debug) disp("transp "+n+" parado"); end
                    delete(speed(n))
                    speed(n) = annotation('textbox', [dx+ddx*n, dy+ddy*n, barx, 0.03], 'string', "vel"+n+":");
                    gsuc(n) = 0; count(n) = 0; waypoints(n,:) = [i1(n) j1(n)];
                else
                    obs = [0 0];
                    if(conflito)
                        obs_count = 0;
                        for m = 1:num_transp
                            if(m~=n)
                                obs_count = obs_count + 1;
                                obs(obs_count,:) = [i1(m) j1(m)];
                            end
                        end
                    end
                    [teta(n), gsuc(n), count(n), linha(n), waypoints(n,:)] = Astar_multi([i1(n) j1(n)], [i2(n) j2(n)], teta(n), desenhar, cor(n), obs);       
                    delete(speed(n));
                    if(gsuc(n)==0)
                        disp(n+" parado")
                        speed(n) = annotation('textbox', [dx+ddx*n, dy+ddy*n, barx, 0.03], 'string', "vel"+n+":");
                    else
                        speed(n) = annotation('textbox', [dx+ddx*n, dy+ddy*n, barx, 0.03], 'string', "vel"+n+": >>>");
                    end
                    dist(n) = dist(n) + gsuc(n);
                end
            end
            pos1(n) = (waypoints(n,1)-1)*num_col + waypoints(n,2);
            if(desenhar_3d)
                %axes(mapa3);
                figure(mapa3.Number)
                %pos_img3(n,w).LineStyle = '-';
                %pos_img3(n,w).LineWidth = 2;
                pos_img3(n,w).Marker = '.';
                pos_img3(n,w).MarkerSize = 8.5;
                pos_img3(n,w+1) = plot3(waypoints(n,2)*grid, waypoints(n,1)*grid, h(n),'o','Color',linha(n).Color);%[n==3 (n==2)/2 n==1]);
                %axes(mapa);
                figure(mapa.Number)
            end
            [i1(n),j1(n)]=pos_to_ij(pos1(n));
            %if(debug) disp("transp "+n+" chegou em "+pos1(n)); disp(" "); end
            
            if(waypoints(n,:) == waypoints_prev(n,:))
                aux_w(n) = aux_w(n) + 1;
            end
            waypoints2(n,w-aux_w(n)) = {[waypoints(n,:) ((loading(n)>0) + 2*(unloading(n)>0))]};
            waypoints_prev(n,:) = waypoints(n,:);
            
            if(pausar)
                if(tempo_pause)
                    pause(tempo_pause);
                else
                    pause;
                end
            end

%             if(sum(finished)==2*num_transp)
%                 break
%             end
%             if(desenhar && (count(n)>0))
%                 children = get(gca, 'children');
%                 delete(children(1:count(n)));
%             end
        else
            if(debug)
                disp("transp "+n+" nao tem atividades");
            end
        end
%         F(w) = getframe(gcf);
%         w=w+1;
    end
    w=w+1;
    if(desenhar)
        for n = 1:num_transp
            %linha(i).Color = [0.65 0.65 0.95];
            if(desenhar_hist)
                linha(n).LineStyle = "--";
            else
                delete(linha(n))
            end
        end
        %children = get(gca, 'children');
        %delete(children(1:count));
    end
end
disp("end1")
% video = VideoWriter("video","MPEG-4");
% video.Quality = 80;
% video.FrameRate = 5;
% open(video);
% writeVideo(video, F);
% close(video);
% disp("end2")

% reduz numero de wp de acordo com colinearidade dos mesmos
% wp_final={};
% for n=1:num_transp
%     dir_anterior = [0 0];
%     wp = waypoints2(n,:);
%     waypoints_new = waypoints2(n,:);
%     k=0;
%     for j = 1:(length(wp)-1)
%         dir_atual = [wp{j+1}(1)-wp{j}(1),wp{j+1}(2)-wp{j}(2)];
%         if((dir_atual == dir_anterior) | (dir_atual==[0 0]))
%             waypoints_new(j-k)=[];
%             k=k+1;
%         end
%         dir_anterior = dir_atual;
%     end
%     wp_final(n,:) = waypoints_new;
% end
% waypoints2 = wp_final

end

function [i1, j1] = pos_to_ij(pos)
    global num_col
    i1=fix(pos/num_col)+1;
    j1=mod(pos,num_col);
    if j1==0
        j1 = num_col; i1 = i1-1;
    end
end

function max_carga = decidir_qtd_carga(n)
    global num_carga k2 debug num_obj_decididos num_obj2
    %if(length(num_carga(n))<k2 || (length(num_carga(n))==1 && num_carga(n)==0))
%     if(num_carga(n,k2(n))>num_obj2-num_obj_decididos-obj_sobra)
%         num_carga(n,k2(n)) = num_obj2-num_obj_decididos-obj_sobra;
%     end
    num_obj_decididos = num_obj_decididos + num_carga(n,k2(n));
    if(num_carga(n,k2(n))>1)
        num_carga(n,k2(n)+1:k2(n)+num_carga(n,k2(n))-1)=0; %se iter k2 pegou x objetos, iter k2+1:k2+x-1 pega 0
    end
    if(debug) disp(n+" decidindo qtd_carga: "+num_carga(n,k2(n))); end
    max_carga = num_carga(n,k2(n));
end

function pos = decidir_carga(n)
    global k2 mov1 obj finished no_inic debug mov_carga
    %global obj_qtd_temp carga_iter
    if(debug) disp("transp "+n+", k2= "+k2(n)); end
    mov1(n,k2(n))=mov_carga(n,k2(n));
    if(mov1(n,k2(n))==0)
        if(debug) disp(n+" demanda atingida, terminou, indo para no_inic"); end 
        finished(n) = 1;
        pos=no_inic(n); %caso nao existam mais obj a serem pegados
    else
        pos=obj(mov1(n,k2(n))); % nova posição
        if(debug) disp("transp "+n+": carga em "+pos); end
        k2(n) = k2(n) + 1;
    end
end

function pos = decidir_descarga(n)
    global k3 debug carga_iter mov2 no_final mov_descarga k2 mov_carga obj_qtd obj2
    %global obj_qtd_temp
    %obj2(n,k3(n)) = sum(obj_qtd(1:mov_carga(n,k2(n)-1)))-obj_qtd_temp(mov_carga(n,k2(n)-1));
    %mov2(n,k3(n)) = mov_descarga(n,obj2(n,k3(n)));
    for i=1:obj_qtd(mov_carga(n,k3(n)))
        obj2(n,k3(n)) = sum(obj_qtd(1:mov_carga(n,k3(n))-1))+i;
        mov2(n,k3(n)) = mov_descarga(n,obj2(n,k3(n)));
        if(mov2(n,k3(n))~=0)
            mov_descarga(n,obj2(n,k3(n)))=0; %evitar que aquele objeto seja selecionado denovo
            break;
        end
    end
    
%     disp("mov_carga(n,k2(n)-1) = "+mov_carga(n,k2(n)-1))
%     disp("obj2(n,k3(n)) = "+obj2(n,k3(n)))
%     disp("mov2(n,k3(n)) = "+mov2(n,k3(n)))
%     disp("no_final(mov2(n,k3(n))) = "+no_final(mov2(n,k3(n))))
    pos=no_final(mov2(n,k3(n))); % nova posição
    if(debug) disp("transp "+n+": descarga em "+pos); end
    k3(n) = k3(n) + 1;
end
