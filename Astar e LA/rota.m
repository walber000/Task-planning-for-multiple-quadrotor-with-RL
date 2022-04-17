% Learning automata para decisão de melhor sequência de captura de objetos
% em diferentes posições.
% Adaptado de rl_lab.m (LA para soluçao de labirinto) do prof. Cairo.
% Walber Lima P. Junior, 05/09/19

function [dist_final, waypoints] = rota(prob1, prob2, p_carga)

%clear all
%close all;
%rand('state',0);

desenhar = 1; pausar = 1; tempo_pause = 0;
if(desenhar) plotMap; end
waypoints = [];
pause;

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

for i = 1:num_lin
    for j = 1:num_col
        % if map(i,j) == 0 -> espaço vazio
        % if map(i,j) == 1 -> obstaculo
        if map(i,j) == 2  %Nó inicial no mapa
            i_inic = i; j_inic = j;
        end
        if map(i,j) == 3  %Objeto no mapa
            num_obj = num_obj + 1; obj(num_obj,:) = num_col*(i-1)+j;
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
Nmov1 = num_obj;
Nmov2 = num_zonas;

%movimentos de carga
for i = 1:Nmov1
    if(max(prob1(i,:))<prob_conv)
        mov_carga(i) = 0;
    else
        mov_carga(i) = find(prob1(i,:)==max(prob1(i,:)));
    end
end

%movimentos de descarga
for i = 1:Nmov1
    if(max(prob2(i,:))<prob_conv)
        mov_descarga(i) = 0;
    else
        mov_descarga(i) = find(prob2(i,:)==max(prob2(i,:)));
    end
end
mov_carga
mov_descarga
%quantidade de carga
for i = 1:Nmov1-1
    num_carga(i) = min(find(p_carga(i,:)==max(p_carga(i,:)))); %min evita probs iguais
end
%carga = max(num_carga);
carga_zona = zeros(1,Nmov2);
t = cell(1,Nmov2);

%movimentação
pos1 = no_inic;
dist_final = 0;
k2 = 1;
tc = 0.4;

Nmov1 = sum(mov_carga~=0);
while(k2 <= Nmov1)
    carga_iter = 0;
    if(k2 == Nmov1)
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
        
        [gsuc, count] = Astar([i1 j1], [i2 j2], desenhar, 1);
        dist_final = dist_final + gsuc;
        pos1 = pos2;

        if(pausar)
            if(tempo_pause)
                pause(tempo_pause);
            else
                pause;
            end
        end
        
        text(j2,i2-0.025,"X",'HorizontalAlignment','center','FontSize',16,'color','red')
        
        if(desenhar)
            children = get(gca, 'children');
            delete(children(2:count+1));
        end
        carga_iter = carga_iter + 1;
        
        k2 = k2 + 1;
    end
    
    % Descarga:
    k3 = k2 - 1;
    
    if(desenhar) title("mov descarga "+k3+", zona "+mov_descarga(mov_carga(k3))); end
    
    pos2=no_final(mov_descarga(mov_carga(k3))); % nova posição
    
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
    
    if(~isa(t{mov_descarga(mov_carga(k3))},'double'))
        delete(t{mov_descarga(mov_carga(k3))})
    end
    carga_zona(mov_descarga(mov_carga(k3))) = carga_zona(mov_descarga(mov_carga(k3))) + 1; 
    texto=sprintf('%d',carga_zona(mov_descarga(mov_carga(k3))));
    t{mov_descarga(mov_carga(k3))}=text(j2+0.3,i2-0.3,texto,'HorizontalAlignment','center','Color','black','FontSize',11);  
    
    waypoints(end+1,:) = [i2 j2];
    
    [gsuc, count] = Astar([i1 j1], [i2 j2], desenhar, 2);
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
        children = get(gca, 'children');
        delete(children(1:count));
    end
end
end