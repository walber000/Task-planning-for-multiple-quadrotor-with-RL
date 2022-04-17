% Learning automata para decisão de melhor sequência de captura de objetos
% em diferentes posições.
% Adaptado de rl_lab.m (LA para soluçao de labirinto) do prof. Cairo.
% Walber Lima P. Junior, 05/09/19

clear all; close all;
desenhar = 0; pausar = 0; tempo_pause = 0;
if(desenhar) plotMap; end

% PASSO 1: leitura do labirinto no arquivo "lab_traj2.txt" e definicao do vetor
% de recompensas LAB_REC
% 0 = estado permitido (célula livre)
% 1 = estado não permitido (obstáculo)
% 2 = estado/posição inicial (robô)
% 3 = estados a serem alcançados (objetos)
% 4 = ponto de entrega de objetos

file = fopen('lab_traj2.txt','r');
size = fscanf(file,'%d',[1 2]);
num_lin = size(1,1);
num_col = size(1,2);
map = fscanf(file,'%d',[num_col num_lin]);
map = map';
num_obj = 0; %iterador para numero de objetos
num_zonas = 0; %iterador para numero de zonas de descarga

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
% Definicao dos parametros do metodo Learning Automata
NIterMax=1000; % no. de iterações total de aprendizado
NStepMax=30; %no. de passos/iteraçoes para começar a atualizar probabilidades
prob_conv = 0.98; %probab para considerar convergencia alcançada
neta=0.1;     % taxa de aprendizado
cargo = 1; %numero de objetos que podem ser carregados pelo drone %NOT WORKING
ref_s = 2;
ref_i = -2;

%ordem correta = [1/2 3/4 5] %primeiro 1 e 2 (qualquer ordem), depois 3 e
%4, depois 5
ordem = [1 2 0 0 0; 1 2 0 0 0; 0 0 3 4 0; 0 0 3 4 0; 0 0 0 0 5];
ordem = [1 2 3 4 5; 1 2 3 4 5; 1 2 3 4 5; 1 2 3 4 5; 1 2 3 4 5];
erros_ordem = [];
%objetos aceitos por cada zonar
tipo_zona = [1 0 0 0 0; 0 2 0 0 0; 0 0 3 0 0; 0 0 0 4 5];
%tipo_zona = [1 2 3 4 5; 1 2 3 4 5; 1 2 3 4 5; 1 2 3 4 5];
erros_zona = [];

Ncel = num_col * num_lin;
Nmov1 = num_obj;
Nmov2 = num_zonas;

%Definiçao dos vetores de probabilidades para i-esima açao de carga
%linhas: numero de açoes de carga a serem realizadas
%colunas: qual objeto será capturado na i-esima açao de carga
for i = 1:Nmov1 %matriz Nmov1 x Nmov1
    prob1(i,:) = (1/Nmov1)*ones(1,Nmov1);
end
%Definiçao dos vetores de probabilidades para descarga de cada peça
%linhas: objeto que foi capturado
%colunas: zona de descarga onde será descarregado o objeto i
for i = 1:Nmov1 %matriz Nmov1 x Nmov2
    prob2(i,:) = (1/Nmov2)*ones(1,Nmov2);
end

k=1;
dist_min = 999;
while (k <= NIterMax && (sum(max(prob1,[],2)<prob_conv)~=0 || sum(max(prob2,[],2)<prob_conv)~=0))
    disp("iter "+k)
    %probab acumulada das ações de carga
    prob_sum1(1,1)=prob1(1,1);
    for j=2:Nmov1
        prob_sum1(1,j)=prob_sum1(1,j-1)+prob1(1,j);
    end
    %probab acumulada das ações de descarga
    for i=1:Nmov1
        prob_sum2(i,1)=prob2(i,1);
        for j=2:Nmov2
            prob_sum2(i,j)=prob_sum2(i,j-1)+prob2(i,j);
        end
    end
    
    pos1=no_inic;
    prob_temp1 = prob1;
    prob_temp2 = prob2;
    dist = 0;
    k2 = 1; %iterador de carga
    e_ordem = 0;
    e_zona = 0;
    while(k2 <= Nmov1)
        cargo_iter = 0;
        while((cargo_iter < cargo) && k2 <= Nmov1)
            %Carga
            title("iter "+k+", mov "+k2)
            %escolha do objeto a ser carregado
            p=rand;
            q=sum(p<=prob_sum1(k2,:));
            mov1(k2)=Nmov1-q+1; % no. do movimento: 1 a Nmov
            pos2=obj(mov1(k2)); % nova posição

            if(~ismember(mov1(k2),ordem(k2,:)))
                e_ordem = 1; %erro de ordem
                erros_ordem(end+1) = k; %salva iteraçao que deu erro
                if(k2<Nmov1)
                    mov1(k2+1:Nmov1) = 0;
                    mov2(k2+1:Nmov1) = 0;
                end
                dist = 0;
                break
            end
            
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
            [gsuc, count] = Astar([i1 j1], [i2 j2], desenhar, 1);
            dist = dist + gsuc;
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
            cargo_iter = cargo_iter + 1;

            if k2 < Nmov1
                prob_temp1(k2+1:end,mov1(k2)) = 0; %zera probabilidade da carga que ja foi capturada
                prob_temp1(k2+1,:)=prob_temp1(k2+1,:)/sum(prob_temp1(k2+1,:)); % normalização
                prob_sum1(k2+1,1)=prob_temp1(k2+1,1);
                for j=2:Nmov1
                    prob_sum1(k2+1,j)=prob_sum1(k2+1,j-1)+prob_temp1(k2+1,j);
                end
            end
        end
        
        % Descarga:
        if(e_ordem)
            break
        end
        p=rand;
        q=sum(p<=prob_sum2(mov1(k2),:));
        mov2(k2)=Nmov2-q+1; % no. do movimento: 1 a Nmov
        pos2=no_final(mov2(k2)); % nova posição

        if(~ismember(mov1(k2),tipo_zona(mov2(k2),:)))
            e_zona = 1; %erro de zona
            erros_zona(end+1) = k; %salva iteraçao que deu erro
            if(k2<Nmov1)
                mov1(k2+1:Nmov1) = 0;
                mov2(k2+1:Nmov1) = 0;
            end
            dist = 0;
            break
        end

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
        [gsuc, count] = Astar([i1 j1], [i2 j2], desenhar, 2);
        dist = dist + gsuc;
        pos1 = pos2;
        
        k2 = k2 + 1;
        
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
    dist_hist(k) = dist;
    mov1_hist(k,:) = mov1;
    mov2_hist(k,:) = mov2;
    if((dist < dist_min) && dist~=0)
        dist_min = dist;
    end
    best(k)=dist_min;
    %vetor de iteraçoes
    x(k)=k;

    %Calculo do reforço
    if(~e_ordem && ~e_zona)
        erros = cat(2, erros_ordem, erros_zona);
        if k >= NStepMax+length(erros)
            %numero de iteraçoes para compor a media
            num_iter = NStepMax;
            num_erros = sum(ismember(erros,x((end-num_iter+1):end)));
            while(num_iter - num_erros < NStepMax)
                num_iter = num_iter+1;
                num_erros = sum(ismember(erros,x((end-num_iter+1):end)));
            end
            dist_media = sum(dist_hist(end-num_iter+1:end))/NStepMax;
            reforco(k) = (dist_media - dist)/(dist_media - dist_min);
            if(reforco(k)>0) reforco(k) = min(reforco(k)*ref_s,ref_s);
            else reforco(k) = max(-reforco(k)*ref_i,ref_i); end
            if dist_media == dist_min
                reforco(k) = 0;
            end
            %if dist == dist_min
            %    reforco(k)=reforco(k)*2;
            %end
            %Atualização das prob de carga
            for i=1:Nmov1
                prob1(i,mov1(i))=(1+neta*reforco(k))*prob1(i,mov1(i));
                if prob1(i,mov1(i)) < 0
                   prob1(i,mov1(i))=0;
                end
                prob1(i,:)=prob1(i,:)/sum(prob1(i,:)); % normalização
            end
            %Atualização das prob de descarga
            for i=1:Nmov1
                prob2(mov1(i),mov2(i))=(1+neta*reforco(k))*prob2(mov1(i),mov2(i));
                if prob2(mov1(i),mov2(i)) < 0
                   prob2(mov1(i),mov2(i))=0;
                end
                prob2(mov1(i),:)=prob2(mov1(i),:)/sum(prob2(mov1(i),:)); % normalização
            end
        else
            reforco(k) = 0;
        end
    else
        reforco(k) = -2;
        if(e_ordem)
            prob1(k2,mov1(k2))=(1+neta*reforco(k))*prob1(k2,mov1(k2));
            if prob1(k2,mov1(k2)) < 0
               prob1(k2,mov1(k2))=0;
            end
            prob1(k2,:)=prob1(k2,:)/sum(prob1(k2,:)); % normalização
        end
        if(e_zona)
            prob2(mov1(k2),mov2(k2))=(1+neta*reforco(k))*prob2(mov1(k2),mov2(k2));
            if prob2(mov1(k2),mov2(k2)) < 0
               prob2(mov1(k2),mov2(k2))=0;
            end
            prob2(mov1(k2),:)=prob2(mov1(k2),:)/sum(prob2(mov1(k2),:)); % normalização
        end
    end
    
    prob1
    prob2

    %y(k,1)=mov(1);
    %y(k,2)=mov(2);
    %y(k,3)=mov(3);
    k = k + 1;
    clear p q;
end
disp("total iter: "+k)
plotMap
prob1
prob2
figure(1)
%subplot(3,1,1)
%plot(x,y(:,1))
%subplot(3,1,2)
%plot(x,y(:,2))
%subplot(3,1,3)
%plot(x,y(:,3))
%figure(2)
plot(setdiff(x,erros),dist_hist(dist_hist~=0 & dist_hist~=999))
hold on
plot(x(best~=999),best(best~=999),'g')
figure(2)
plot(x,reforco)