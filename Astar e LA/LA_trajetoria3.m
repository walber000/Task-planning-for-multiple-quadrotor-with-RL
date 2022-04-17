% Learning automata para decisão de melhor sequência de captura de objetos
% em diferentes posições.
% Adaptado de rl_lab.m (LA para soluçao de labirinto) do prof. Cairo.
% Walber Lima P. Junior, 05/09/19

clear all; close all;
desenhar = 0; pausar = 0; tempo_pause = 0.2;
if(desenhar) plotMap; end

% PASSO 1: leitura do labirinto no arquivo "lab_traj.txt" e definicao do vetor
% de recompensas LAB_REC
% 0 = estado permitido (célula livre)
% 1 = estado não permitido (obstáculo)
% 2 = estado/posição inicial (robô)
% 3 = estados a serem alcançados (objetos)
% 4 = ponto de entrega de objetos

file = fopen('lab_traj.txt','r');
size = fscanf(file,'%d',[1 2]);
num_lin = size(1,1);
num_col = size(1,2);
map = fscanf(file,'%d',[num_col num_lin]);
map = map';
num_obj = 0; %iterador para numero de objetos

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
            i_final = i; j_final = j;
        end
    end
end

no_inic = num_col*(i_inic-1)+j_inic;
no_final = num_col*(i_final-1)+j_final;

% Definicao dos parametros do metodo Learning Automata
NIterMax=1000; % no. de iterações total de aprendizado
NStepMax=20; %no. de passos/iteraçoes para começar a atualizar probabilidades
prob_conv = 0.95; %probab para considerar convergencia alcançada
neta=0.1;     % taxa de aprendizado
cargo = 1; %numero de objetos que podem ser carregados pelo drone

%ordem correta = [1/2 3/4 5] %primeiro 1 e 2 (qualquer ordem), depois 3 e
%4, depois 5
ordem = [1 2 0 0 0; 1 2 0 0 0; 3 4 0 0 0; 3 4 0 0 0; 5 0 0 0 0];
erros_ordem = [];

Ncel = num_col * num_lin;
Nmov = num_obj;

%Definiçao dos vetores de probabilidades para i-esima açao de carga
for i = 1:Nmov
    prob(i,:) = (1/Nmov)*ones(1,Nmov);
end

k=1;
dist_min = 999;
while (k <= NIterMax && sum(max(prob,[],2)<prob_conv)~=0 )
    disp("iter "+k)
    title("iter "+k)
    prob_sum(1,1)=prob(1,1);
    for j=2:Nmov
        prob_sum(1,j)=prob_sum(1,j-1)+prob(1,j);
    end
    pos(1)=no_inic;
    prob_temp = prob;
    dist = 0;
    k2 = 1;
    e_ordem = 0;
    while(k2 <= Nmov)
        cargo_iter = 0;
        while((cargo_iter < cargo) && k2 <= Nmov)
            %Carga
            title("iter "+k+", mov "+k2)
            %escolha do objeto a ser carregado
            p=rand;
            q=sum(p<=prob_sum(k2,:));
            mov(k2)=Nmov-q+1; % no. do movimento: 1 a Nmov
            pos(k2+1)=obj(mov(k2)); % nova posição

            if(~ismember(mov(k2),ordem(k2,:)))
                e_ordem = 1; %erro de ordem
                erros_ordem(end+1) = k; %salva iteraçao que deu erro
                if(k2<Nmov) mov(k2+1:Nmov) = 0; end
                dist = 0;
                break
            end
            
            i1=fix(pos(k2)/num_col)+1;
            j1=mod(pos(k2),num_col);
            if j1==0
                j1 = num_col; i1 = i1-1;
            end
            i2=fix(pos(k2+1)/num_col)+1;
            j2=mod(pos(k2+1),num_col);
            if j2==0
                j2 = num_col; i2 = i2-1;
            end
            [gsuc, count] = Astar([i1 j1], [i2 j2], desenhar, 1);
            dist = dist + gsuc;
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
            cargo_iter = cargo_iter + 1;

            k2 = k2 + 1;
            if k2 <= Nmov
                prob_temp(k2:end,mov(k2-1)) = 0; %zera probabilidade da carga que ja foi capturada
                prob_temp(k2,:)=prob_temp(k2,:)/sum(prob_temp(k2,:)); % normalização
                prob_sum(k2,1)=prob_temp(k2,1);
                for j=2:Nmov;
                    prob_sum(k2,j)=prob_sum(k2,j-1)+prob_temp(k2,j);
                end;
            end
        end
        
        % Descarga:
        if(e_ordem)
            break
        end
        [gsuc, count] = Astar([i2 j2], [i_final j_final],desenhar, 2);
        dist = dist + gsuc;
        pos(k2) = no_final;
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
    
    % Armazena a distancia total da iteração
    dist_hist(k,:) = dist;
    mov_hist(k,:) = mov;
    if((dist < dist_min) && dist~=0)
        dist_min = dist;
    end
    
    %vetor de iteraçoes
    x(k)=k;
    
    %calculo do reforço e aprendizado
    if(~e_ordem)
        if k >= NStepMax+length(erros_ordem)
            %numero de iteraçoes para compor a media
            num_iter = NStepMax;
            num_erros = sum(ismember(erros_ordem,x((end-num_iter+1):end)));
            while(num_iter - num_erros < NStepMax)
                num_iter = num_iter+1;
                num_erros = sum(ismember(erros_ordem,x((end-num_iter+1):end)));
            end
            dist_media = sum(dist_hist(end-num_iter+1:end))/NStepMax;
            reforco(k) = sign((dist_media - dist)/(dist_media - dist_min));
            if dist_media == dist_min
                reforco(k) = 1;
            end
            for i=1:Nmov
                prob(i,mov(i))=(1+neta*reforco(k))*prob(i,mov(i));
                if prob(i,mov(i)) < 0
                   prob(i,mov(i))=0;
                end
                prob(i,:)=prob(i,:)/sum(prob(i,:)); % normalização
            end
        else
            reforco(k) = 0;
        end
    else
        reforco(k) = -2;
        prob(k2,mov(k2))=(1+neta*reforco(k))*prob(k2,mov(k2));
        if prob(k2,mov(k2)) < 0
           prob(k2,mov(k2))=0;
        end
        prob(k2,:)=prob(k2,:)/sum(prob(k2,:)); % normalização
    end
    prob
    %y(k,1)=mov(1);
    %y(k,2)=mov(2);
    %y(k,3)=mov(3);5
    k = k + 1;
    clear p q;
end
figure(1)
%subplot(3,1,1)
%plot(x,y(:,1))
%subplot(3,1,2)
%plot(x,y(:,2))
%subplot(3,1,3)
%plot(x,y(:,3))
%figure(2)
plot(x,reforco)
