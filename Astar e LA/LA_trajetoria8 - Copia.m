% Learning automata para decisão de melhor sequência de captura de objetos
% em diferentes posições.
% Adaptado de rl_lab.m (LA para soluçao de labirinto) do prof. Cairo.
% Walber Lima P. Junior, 05/09/19

clear all; close all;

t1=now;

%plotMap
%[gsuc, count, ~, ~] = Astar([7 3], [10 10], 1, 1);
%return

%rand('state',0);
desenhar = 0; pausar = 0; tempo_pause = 0;
if(desenhar) plotMap; end

metodo_atualizacao = 1; %0 = LRI, pag15 Sastry, 1 = padrao igual do rl_lab.m
metodo_normalizacao = 1; %0 = pag15 Sastry, 1 = normalizacao padrao

% PASSO 1: leitura do labirinto no arquivo "activemap.txt" e definicao do vetor
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
neta=0.12;     % taxa de aprendizado
ref_s = 2;
if(metodo_atualizacao)
    ref_i = -2;
else
    ref_i = 0;
end

for i = 1:num_obj %objeto generico (quantidade = 1)
    obj_qtd(i)=1;
end
%obj_qtd = [1 1];
%obj_qtd = [1 1 3 2 1];
num_obj2 = sum(obj_qtd); %quantidade de material

%ordem correta = [1/2 3/4 5] %primeiro 1 e 2 (qualquer ordem), depois 3 e
%4, depois 5
for i = 1:num_obj2 %ordem generica (qualquer ordem)
    ordem(1:num_obj2,i)=i;
end
%ordem = [1 2 0 0 0; 1 2 0 0 0; 0 0 3 4 0; 0 0 3 4 0; 0 0 0 0 5];
%ordem = [0 0 3 4 5; 0 0 3 4 5; 0 0 3 4 5; 1 2 0 0 0; 1 2 0 0 0];
%ordem = [0 0 3 0 0; 0 0 0 4 5; 0 0 0 4 5; 1 0 0 0 0; 0 2 0 0 0];
erros_ordem = [];

%objetos aceitos por cada zona
for i = 1:num_obj2 %zonas generica (qualquer objeto nas zonas)
    tipo_zona(1:num_zonas,i)=i;
end
%tipo_zona = [1 0 0 0 0; 0 2 0 0 0; 0 0 3 0 0; 0 0 0 4 5];
%tipo_zona = [1 2 0 4 0; 1 2 0 4 0; 0 0 3 0 5; 0 0 3 0 5];
erros_zona = [];
erros = [];

for i = 1:num_zonas %demanda generica (demanda max para todas as zonas)
    demanda_zona(i)=num_obj2;
end
%demanda_zona = [1 1 1 1]; %nao colocar 0, ao inves, apagar zona
%demanda_full = 0; %torna-se 1 quando a demanda de todas as zonas é atendida
obj_sobra = max(num_obj2 - sum(demanda_zona), 0); %numero de objetos que sobrarão

Ncel = num_col * num_lin;

%Definiçao dos vetores de probabilidades para i-esima açao de carga
%linhas: numero de açoes de carga a serem realizadas
%colunas: qual objeto será capturado na i-esima açao de carga
for i = 1:num_obj2 % numero de açoes de carga X numero de zonas de carga
    prob1(i,:) = (1/num_obj)*ones(1,num_obj);
end
%Definiçao dos vetores de probabilidades para descarga de cada peça
%linhas: objeto que foi capturado
%colunas: zona de descarga onde será descarregado o objeto i
for i = 1:num_obj2 % numero de objetos X numero de zonas de descarga
    prob2(i,:) = (1/num_zonas)*ones(1,num_zonas);
end

carga = 3; %numero de objetos que podem ser carregados pelo drone
%Definiçao dos vetores de probabilidades para quantidade de carga
for i = 1:(num_obj2-1)
    %if(num_obj2
    p_carga(i,:) = (1/carga)*ones(1,carga);
end

k=1;
dist_min = 999;
while (k <= NIterMax && (sum(max(prob1,[],2)<prob_conv)>obj_sobra || sum(max(prob2,[],2)<prob_conv)>obj_sobra))
    disp("iter "+k)
    %title("iter "+k)
    %probab acumulada das ações de carga
    prob_sum1(1,1)=prob1(1,1);
    for j=2:num_obj
        prob_sum1(1,j)=prob_sum1(1,j-1)+prob1(1,j);
    end
    %probab acumulada das ações de descarga
    for i=1:num_obj2
        prob_sum2(i,1)=prob2(i,1);
        for j=2:num_zonas
            prob_sum2(i,j)=prob_sum2(i,j-1)+prob2(i,j);
        end
    end
    %probab acumulada da quantidade de carga
    for i=1:num_obj2-1
        p_carga_sum(i,1)=p_carga(i,1);
        for j=2:carga
            p_carga_sum(i,j)=p_carga_sum(i,j-1)+p_carga(i,j);
        end
    end
    pos1=no_inic;
    prob_temp1 = prob1;
    prob_temp2 = prob2;
    dist = 0;
    carga_zona = zeros(1,num_zonas);
    k2 = 1; %iterador de carga
    e_ordem = 0;
    e_zona = 0;
    obj_qtd_temp = obj_qtd;
    %p_carga_sum_temp = p_carga_sum;
    num_carga = [];
    mov2 = [];
    while(k2 <= num_obj2)
        carga_iter = 0;
        if(length(num_carga)<k2)
            if(k2 == num_obj2)
                num_carga(k2)=1;
            else
                p=rand;
                q=sum(p<=p_carga_sum(k2,:));
                num_carga(k2)=carga-q+1;
                if(num_carga(k2)>1)
                    num_carga(k2+1:k2+num_carga(k2)-1)=0; %se iter k2 pegou x objetos, iter k2+1:k2+x-1 pega 0
                end
            end
        end
        if(num_carga(k2)~=0)
            max_carga = num_carga(k2);
        end
        while((carga_iter < max_carga) && k2 <= num_obj2)
            %Carga
            title("iter "+k+", mov "+k2)
            %escolha do objeto a ser carregado
            p=rand;
            q=sum(p<=prob_sum1(k2,:));
            mov1(k2)=num_obj-q+1; % no. do movimento: 1 a Nmov
            pos2=obj(mov1(k2)); % nova posição
            %if (pos1==pos2)
            %    disp("erro: pos1=pos2")
            %    return
            %end

            if(~ismember(mov1(k2),ordem(k2,:)))
                e_ordem = 1; %erro de ordem
                erros_ordem(end+1) = k; %salva iteraçao que deu erro
                if(k2<num_obj2)
                    mov1(k2+1:num_obj2) = 0;
                    mov2(k2+1:num_obj2) = 0;
                    num_carga(k2+1:num_obj2) = 0;
                    obj2(k2+1:num_obj2) = 0;
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
            
            if(pos1==pos2)
                gsuc = 0; count = 0;
            else
                [gsuc, count, ~, ~] = Astar([i1 j1], [i2 j2], desenhar, 1);
            end
            dist = dist + gsuc;
            pos1 = pos2;
            
            if(pausar)
                if(tempo_pause)
                    pause(tempo_pause);
                else
                    pause;
                end
            end
            if(desenhar && (count>0))
                children = get(gca, 'children');
                delete(children(1:count));
            end
            carga_iter = carga_iter + 1;
            obj_qtd_temp(mov1(k2)) = obj_qtd_temp(mov1(k2)) - 1;
            if k2 < num_obj2
                if(obj_qtd_temp(mov1(k2))<1)
                    prob_temp1(k2+1:end,mov1(k2)) = 0; %zera probabilidade da carga que ja foi capturada
                end
                prob_temp1(k2+1,:)=prob_temp1(k2+1,:)/sum(prob_temp1(k2+1,:)); % normalização
                prob_sum1(k2+1,1)=prob_temp1(k2+1,1);
                for j=2:num_obj
                    prob_sum1(k2+1,j)=prob_sum1(k2+1,j-1)+prob_temp1(k2+1,j);
                end
            end
            k2 = k2 + 1;
        end
        
        % Descarga:
        if(e_ordem)
            break
        end
        k3 = k2 - 1;
        aux = 0;
        p=rand;
        %caso esteja com mais de 1 obj, escolher zona de descarga pela
        %media de probabilidades dos objetos com as zonas
        prob_mix = zeros(1,num_zonas);
        for i=(k3-carga_iter+1):k3
            if(carga_iter>1 && i~=k3)
                j = i;
                while(mov1(j)==mov1(j+1))
                    aux = aux + 1; j = j + 1;
                    if(j==k3) break; end
                end
            else
                aux=0;
            end
            obj2(i) = sum(obj_qtd(1:mov1(i)))-obj_qtd_temp(mov1(i))-aux;
            prob_mix = prob_mix + prob_sum2(obj2(i),:);
        end
        prob_mix = prob_mix/carga_iter;
        q=sum(p<=prob_mix);
        mov2(k3)=num_zonas-q+1; % no. do movimento: 1 a num_zonas
        pos2=no_final(mov2(k3)); % nova posição
        
        %checa se os objetos foram colocados na zona correta
        if(~prod(ismember(mov1(k3-carga_iter+1:k3),tipo_zona(mov2(k3),:))))
            %disp("mov1: "+mov1(k3-carga_iter+1:k3))
            %disp("mov2: "+mov2(k3))
            e_zona = 1; %erro de zona
            erros_zona(end+1) = k; %salva iteraçao que deu erro
            if(k3<num_obj2)
                mov1(k2:num_obj2) = 0;
                mov2(k2:num_obj2) = 0;
                num_carga(k2:num_obj2) = 0;
                obj2(k2:num_obj2) = 0;
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
        [gsuc, count, ~, ~] = Astar([i1 j1], [i2 j2], desenhar, 2);
        dist = dist + gsuc;
        pos1 = pos2;
        
        carga_zona(mov2(k3)) = carga_zona(mov2(k3)) + 1;
        if(carga_zona(mov2(k3))==demanda_zona(mov2(k3)))
            prob_temp2(:,mov2(k3)) = 0; %zona nao aceita mais cargas
            if(sum(prob_temp2(i,:))==0) %caso todas as demandas sejam atendidas
                %demanda_full = 1;
                mov1(k2:num_obj2) = 0;
                mov2(k2:num_obj2) = 0;
                num_carga(k2:num_obj2) = 0;
                obj2(k2:num_obj2) = 0;
                break
            end
            for i=1:num_obj2
                prob_temp2(i,:)=prob_temp2(i,:)/sum(prob_temp2(i,:)); % normalização
            end
            for i=1:num_obj2
                prob_sum2(i,1)=prob_temp2(i,1);
                for j=2:num_zonas
                    prob_sum2(i,j)=prob_sum2(i,j-1)+prob_temp2(i,j);
                end
            end
        end
        
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
    carga_hist(k,:) = num_carga;
    if((dist < dist_min) && dist~=0)
        dist_min = dist;
    end
    best(k)=dist_min;
    %vetor de iteraçoes
    x(k)=k;

    %Calculo do reforço
    if(~e_ordem && ~e_zona)
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
            for i=1:num_obj2
                if(mov1(i)~=0)
                    if(metodo_atualizacao)
                        prob1(i,mov1(i))=(1+neta*reforco(k))*prob1(i,mov1(i));
                    else
                        prob1(i,mov1(i))=prob1(i,mov1(i)) + neta*reforco(k)*(1-prob1(i,mov1(i)));
                    end
                    if prob1(i,mov1(i)) < 0
                       prob1(i,mov1(i))=0;
                    end
                    if(metodo_normalizacao)
                        prob1(i,:)=prob1(i,:)/sum(prob1(i,:)); % normalização
                    else
                        for j=1:num_obj
                            if(j~=mov1(i))
                                prob1(i,j)=prob1(i,j) - neta*reforco(k)*prob1(i,j);
                            end
                        end
                    end
                end
            end

            %Atualização das prob de descarga
            z=0;
            for i=1:num_obj2
                if(mov2(i)==0)
                    z = z + 1;
                else
                    if(metodo_atualizacao)
                        prob2(obj2((i-z):i),mov2(i))=(1+neta*reforco(k))*prob2(obj2((i-z):i),mov2(i));
                    else
                        prob2(obj2((i-z):i),mov2(i))=prob2(obj2((i-z):i),mov2(i))+neta*reforco(k)*(1-prob2(obj2((i-z):i),mov2(i)));
                    end
                    for j = (i-z):i
                        if prob2(obj2(j),mov2(i)) < 0
                           prob2(obj2(j),mov2(i))=0;
                        end
                        %normalização
                        if(metodo_normalizacao)
                            prob2(obj2(j),:)=prob2(obj2(j),:)/sum(prob2(obj2(j),:));
                        else
                            for m=1:num_zonas
                                if(m~=mov2(i))
                                    prob2(obj2(j),m)=prob2(obj2(j),m) - neta*reforco(k)*prob2(obj2(j),m);
                                end
                            end
                        end
                    end
                    z=0;
                end
            end
            %aprendizado de p_carga (num_carga)
            if(carga>1)
                for i=1:num_obj2-1
                    if(num_carga(i)~=0)
                        p_carga(i,num_carga(i))=(1+neta*reforco(k))*p_carga(i,num_carga(i));

                        if p_carga(i,num_carga(i)) < 0
                           p_carga(i,num_carga(i)) = 0;
                        end
                        p_carga(i,:)=p_carga(i,:)/sum(p_carga(i,:)); % normalização
                    end
                end
            end
        else
            reforco(k) = 0;
        end
    else
        erros = cat(2, erros_ordem, erros_zona);
        reforco(k) = -8;
        if(e_ordem)
            prob1(k2,mov1(k2))=(1+neta*reforco(k))*prob1(k2,mov1(k2));
            if prob1(k2,mov1(k2)) < 0
               prob1(k2,mov1(k2))=0;
            end
            prob1(k2,:)=prob1(k2,:)/sum(prob1(k2,:)); % normalização
        end
        if(e_zona)
            %diminui prob de objeto ser descarregado na zona incorreta
            index = find(ismember(obj2(k3-carga_iter+1:k3),tipo_zona(mov2(k3),:))==0);
            index = k3-carga_iter+index;
            for i = 1:length(index)
                prob2(obj2(index(i)),mov2(k3))=(1+neta*reforco(k))*prob2(obj2(index(i)),mov2(k3));
                %prob2(mov1(index(i)),mov2(k3)) = 0.001;
                if prob2(obj2(index(i)),mov2(k3)) < 0
                   prob2(obj2(index(i)),mov2(k3))=0;
                end
                prob2(obj2(index(i)),:)=prob2(obj2(index(i)),:)/sum(prob2(obj2(index(i)),:)); % normalização
            end
            %diminui prob de carregar mais de 1 objeto
%             if(carga_iter>1)
%                 i=k3-carga_iter+1;
%                 p_carga(i,num_carga(i))=(1+neta*reforco(k))*p_carga(i,num_carga(i));
%                 if p_carga(i,num_carga(i)) < 0
%                    p_carga(i,num_carga(i)) = 0;
%                 end
%                 p_carga(i,:)=p_carga(i,:)/sum(p_carga(i,:)); % normalização
%             end
        end
    end
    prob1
    prob2
    if(carga>1)
        p_carga
    end
    
    aut1 = max(max(prob1,[],2));
    aut2 = max(max(prob2,[],2));
    Tcgmin(k) = min(aut1,aut2);
    Tcgmax(k) = max(aut1,aut2);
    Tcgmed(k) = (aut1+aut2)/2;
    
    clear p q;
    k = k + 1;
    %y(k,1)=mov(1);
    %y(k,2)=mov(2);
    %y(k,3)=mov(3);
end

t2=now;
elapsed_days1 = t2 - t1;
elasped_second1 = elapsed_days1 *24*60*60;
disp("tempo total: "+elasped_second1)
disp("n_iter: "+k)
disp("tempo/iter: "+(elasped_second1/k))

plotMap
prob1
prob2
if(carga>1)
    p_carga
end

%subplot(3,1,1)
%plot(x,y(:,1))
%subplot(3,1,2)
%plot(x,y(:,2))
%subplot(3,1,3)
%plot(x,y(:,3))
%figure(2)

figure(1)
plot(x(best~=999),best(best~=999),'g')
hold on
plot(setdiff(x,erros),dist_hist(dist_hist~=0 & dist_hist~=999))

figure(2)
plot(x,reforco)

figure(3)
plot(x, Tcgmin,'b')
hold on
plot(x, Tcgmed,'r')
plot(x, Tcgmax,'k')
legend("Tcgmin","Tcgmed","Tcgmax")
title("Taxa de convergencia geral (Tcg), atualizacao="+metodo_atualizacao+", normalizacao="+metodo_normalizacao)
grid

[dist,waypoints2]=rota2(prob1,prob2,p_carga, obj_qtd);

