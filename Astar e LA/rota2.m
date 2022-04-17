% Learning automata para decisão de melhor sequência de captura de objetos
% em diferentes posições.
% Adaptado de rl_lab.m (LA para soluçao de labirinto) do prof. Cairo.
% Walber Lima P. Junior, 05/09/19

function [dist_final, waypoints2] = rota2(prob1, prob2, p_carga, obj_qtd, demanda_zona,usar_Astar)

%clear all
%close all;
%rand('state',0);

desenhar = 1; pausar = 1; tempo_pause = 0; desenhar_demanda = 1;
if(desenhar) plotMap; end
waypoints = [];
waypoints2 = {};
% PASSO 1: leitura do labirinto no arquivo "lab_traj2.txt" e definicao do vetor
% de recompensas LAB_REC
% 0 = estado permitido (célula livre)
% 1 = estado não permitido (obstáculo)
% 2 = estado/posição inicial (robô)
% 3 = estados a serem alcançados (objetos)
% 4 = ponto de entrega de objetos

file = fopen('activemap.txt','r');
size = fscanf(file,'%d',[1 2]);
num_lin = size(1,1);
num_col = size(1,2);
map = fscanf(file,'%d',[num_col num_lin]);
map = map';
num_obj = 0; %iterador para numero de objetos
num_zonas = 0; %iterador para numero de zonas de descarga
prob_conv = 0.8;
xh = 1;
xv = 1;

for i = 1:num_lin
    for j = 1:num_col
        % if map(i,j) == 0 -> espaço vazio
        % if map(i,j) == 1 -> obstaculo
        if map(i,j) == 2  %Nó inicial no mapa
            i_inic = i; j_inic = j;
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
for i=1:num_zonas
    no_final(i) = num_col*(zona(i,1)-1)+zona(i,2);
end

Ncel = num_col * num_lin;
num_obj2 = sum(obj_qtd);

%colocar quantidade de material em cada zona de carga
for i=1:num_obj
    texto=sprintf('%d',obj_qtd(i));
    obj_cell{i}=text(j_obj(i)+0.3,i_obj(i)-0.3,texto,'HorizontalAlignment','center','Color','black','FontSize',11);  
end

carga_zona = zeros(1,num_zonas);

%colocar quantidade de material em cada zona de descarga
for i=1:num_zonas
    texto=sprintf('%d',carga_zona(i));
    zona_cell{i}=text(zona(i,2)+0.3-0.1*desenhar_demanda,zona(i,1)-0.3,texto,'HorizontalAlignment','center','Color','black','FontSize',11);  
end

%colocar demanda em cada zona de descarga
if(desenhar_demanda == 1)
    for i=1:num_zonas
        texto=sprintf('/%d',demanda_zona(i));
        text(zona(i,2)+0.5,zona(i,1)-0.3,texto,'HorizontalAlignment','center','Color','black','FontSize',11);  
    end
end

%movimentos de carga
for i = 1:num_obj2
    if(max(prob1(i,:))<prob_conv)
        mov_carga(i) = 0;
    else
        mov_carga(i) = find(prob1(i,:)==max(prob1(i,:)));
    end
end

%movimentos de descarga
for i = 1:num_obj2
    if(max(prob2(i,:))<prob_conv)
        mov_descarga(i) = 0;
    else
        mov_descarga(i) = find(prob2(i,:)==max(prob2(i,:)));
    end
end
mov_carga
mov_descarga
%quantidade de carga
for i = 1:num_obj2-1
    num_carga(i) = min(find(p_carga(i,:)==max(p_carga(i,:)))); %min evita probs iguais
end
%carga = max(num_carga);

%movimentação
pos1 = no_inic;
dist_final = 0;
k2 = 1;
tc = 0.4;

obj_qtd_temp = obj_qtd;
num_obj = sum(mov_carga~=0);

pause;
while(k2 <= num_obj)
    carga_iter = 0;
    if(k2 == num_obj)
        max_carga = 1;
    else
        max_carga = num_carga(k2);
    end
    while(carga_iter < max_carga)
        %Carga
        if(desenhar) title("mov carga "+k2+", obj "+mov_carga(k2)); end
        
        pos2 = obj(mov_carga(k2));
        
        i1=fix(pos1/num_col)+1;
        j1=mod(pos1,num_col);
        if j1==0
            j1 = num_col; i1 = i1-1;
        end
        i2=fix(pos2/num_col)+1;
        j2=mod(pos2,num_col);
        if j2==0
            j2 = num_col; i2 = i2-1;
        end
        
        waypoints(end+1,:) = [i2 j2];
        
        if(pos1==pos2)
            gsuc = 0; count = 0;
        else
            if(usar_Astar)
                [gsuc, count, linha, wp] = Astar([i1 j1], [i2 j2], desenhar, 1);
            else
                dist_hor = abs(j2-j1)*xh;
                dist_vert = abs(i2-i1)*xv;
                gsuc = sqrt(dist_hor^2 + dist_vert^2);
                wp = [i2 j2];
                linha = line([j1 j2],[i1 i2]);
                linha.LineWidth = 2;
                count = 1;
            end
        end
        waypoints2{end+1} = wp;
        dist_final = dist_final + gsuc;
        pos1 = pos2;

        if(pausar)
            if(tempo_pause)
                pause(tempo_pause);
            else
                pause;
            end
        end

        if(desenhar)
            for i = 1:length(linha)
                %linha(i).Color = [0.65 0.65 0.95];
                linha(i).LineStyle = "--";
            end
            %children = get(gca, 'children');
            %delete(children(1:count));
        end
        carga_iter = carga_iter + 1;
        obj_qtd_temp(mov_carga(k2)) = obj_qtd_temp(mov_carga(k2)) - 1;
        
        % marcar X em zona de carga que acabou estoque
        if(obj_qtd_temp(mov_carga(k2))==0)
            text(j2,i2-0.025,"X",'HorizontalAlignment','center','FontSize',16,'color','red')
        end

        %atualizar quantidade de objetos na zona de carga
        texto=sprintf('%d',obj_qtd_temp(mov_carga(k2)));
        delete(obj_cell{mov_carga(k2)})
        obj_cell{mov_carga(k2)}=text(j2+0.3,i2-0.3,texto,'HorizontalAlignment','center','Color','black','FontSize',11);  
        
        k2 = k2 + 1;
    end
    
    % Descarga:
    k3 = k2 - 1;
    
    if(desenhar) title("mov descarga "+k3+", zona "+mov_descarga(mov_carga(k3))); end
    
    aux = 0;
    for i=(k3-carga_iter+1):k3
        if(carga_iter>1 && i~=k3)
            j = i;
            while(mov_carga(j)==mov_carga(j+1))
                aux = aux + 1; j = j + 1;
                if(j==k3) break; end
            end
        else
            aux=0;
        end
        obj2(i) = sum(obj_qtd(1:mov_carga(i)))-obj_qtd_temp(mov_carga(i))-aux;
    end
    pos2=no_final(mov_descarga(obj2(k3))); % nova posição
    
    i1=fix(pos1/num_col)+1;
    j1=mod(pos1,num_col);
    if j1==0
        j1 = num_col; i1 = i1-1;
    end
    i2=fix(pos2/num_col)+1;
    j2=mod(pos2,num_col);
    if j2==0
        j2 = num_col; i2 = i2-1;
    end

    waypoints(end+1,:) = [i2 j2];
    
    if(usar_Astar)
        [gsuc, count, linha, wp] = Astar([i1 j1], [i2 j2], desenhar, 2);
    else
        dist_hor = abs(j2-j1)*xh;
        dist_vert = abs(i2-i1)*xv;
        gsuc = sqrt(dist_hor^2 + dist_vert^2);
        wp = [i2 j2];
        linha = line([j1 j2],[i1 i2]);
        linha.LineWidth = 2;
        linha.Color = [1 0 0];
        count = 1;
    end
    dist_final = dist_final + gsuc;
    pos1 = pos2;
    waypoints2{end+1} = wp;
    
    if(pausar)
        if(tempo_pause)
            pause(tempo_pause);
        else
            pause;
        end
    end
    
    %if(~isa(zona_cell{mov_descarga(obj2(k3))},'double'))
    %    delete(zona_cell{mov_descarga(obj2(k3))})
    %end
    
    %atualizar quantidade de objetos na zona de descarga
    carga_zona(mov_descarga(obj2(k3))) = carga_zona(mov_descarga(obj2(k3))) + carga_iter; 
    texto=sprintf('%d',carga_zona(mov_descarga(obj2(k3))));
    delete(zona_cell{mov_descarga(obj2(k3))})
    zona_cell{mov_descarga(obj2(k3))}=text(j2+0.3-0.1*desenhar_demanda,i2-0.3,texto,'HorizontalAlignment','center','Color','black','FontSize',11);  
    
    if(desenhar)
        for i = 1:length(linha)
            %linha(i).Color = [0.95 0.65 0.65];
            linha(i).LineStyle = "--";
        end
        %children = get(gca, 'children');
        %delete(children(1:count));
    end
end

% reduz numero de wp de acordo com colinearidade dos mesmos
dir_anterior = [0,0];
waypoints_new = waypoints2;
for i = 1:length(waypoints2)
    k=0;
    for j = 1:(length(waypoints2{i})-1)
        dir_atual = [waypoints2{i}(j+1,1)-waypoints2{i}(j,1),waypoints2{i}(j+1,2)-waypoints2{i}(j,2)];
        if(dir_atual == dir_anterior)
            waypoints_new{i}(j-k,:)=[];
            k=k+1;
        end
        dir_anterior = dir_atual;
    end
end
waypoints2 = waypoints_new
end