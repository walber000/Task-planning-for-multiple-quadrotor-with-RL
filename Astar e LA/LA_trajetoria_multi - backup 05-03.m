% Learning automata para decis?o de melhor sequ?ncia de captura de objetos
% em diferentes posi??es.
% Adaptado de rl_lab.m (LA para solu?ao de labirinto) do prof. Cairo.
% Walber Lima P. Junior, 05/09/19

clear all; clc; close all;

global p_carga_sum k2 k3 carga prob_sum1 prob_sum2 prob_temp1 obj_qtd_temp mov1 mov2 k
global num_col obj_qtd num_obj num_obj2 obj num_transp num_zonas no_final carga_zona num_obj_decididos
global demanda_zona pos1 finished no_inic carga_iter debug obj2 prob_temp2 obj_sobra num_carga
global ordem e_ordem erros_ordem count_steps dist

rand('state',0);
desenhar = 0; pausar = 0; tempo_pause = 0; debug = 0; debug_mov = 0; use_log=1;
if(desenhar) plotMap; end

t1=now;
z1=[0 0 0 0];
z2=[0 0 0 0];
z3=[0 0 0 0];
z4=[0 0 0 0];

metodo_atualizacao = 1; %0 = LRI, pag15 Sastry, 1 = padrao igual do rl_lab.m
metodo_normalizacao = 1; %0 = pag15 Sastry, 1 = normalizacao padrao

% PASSO 1: leitura do labirinto no arquivo "activemap.txt" e definicao do vetor
% de recompensas LAB_REC
% 0 = estado permitido (c?lula livre)
% 1 = estado n?o permitido (obst?culo)
% 2 = estado/posi??o inicial (rob?)
% 3 = estados a serem alcan?ados (objetos)
% 4 = ponto de entrega de objetos

file = fopen('activemap_multi.txt','r');
size = fscanf(file,'%d',[1 2]);
num_lin = size(1,1);
num_col = size(1,2);
map = fscanf(file,'%d',[num_col num_lin]);
map = map';
num_obj = 0; %iterador para numero de objetos
num_zonas = 0; %iterador para numero de zonas de descarga
num_transp = 0; %numero de transportadores
log = [0 0 0 0 0 0 0 0 0]; %log de A*: [i1 j1 i2 j2 teta newteta gsuc wp1 wp2]
t_iter = 0;
k_log = 0;
obs = [0 0]; %variavel para considerar outros drones como obstaculos

for i = 1:num_lin
    for j = 1:num_col
        % if map(i,j) == 0 -> espa?o vazio
        % if map(i,j) == 1 -> obstaculo
        if map(i,j) == 2  %N? inicial no mapa
            num_transp = num_transp + 1; i_inic(num_transp) = i; j_inic(num_transp) = j;
        end
        if map(i,j) == 3  %Objeto no mapa
            num_obj = num_obj + 1; obj(num_obj,:) = num_col*(i-1)+j;
        end
        if map(i,j) == 4  %Ponto de entrega no mapa
            num_zonas = num_zonas + 1; zona(num_zonas,1) = i; zona(num_zonas,2) = j;
        end
    end
end

for i=1:num_transp
    no_inic(i) = num_col*(i_inic(i)-1)+j_inic(i);
    cor(i)=2*i-1;
end

for i=1:num_zonas
    no_final(i) = num_col*(zona(i,1)-1)+zona(i,2);
end
% Definicao dos parametros do metodo Learning Automata
NIterMax=10000; % no. de itera??es total de aprendizado
NStepMax=20; %no. de passos/itera?oes para come?ar a atualizar probabilidades
prob_conv = 0.98; %probab para considerar convergencia alcan?ada
neta=0.11;     % taxa de aprendizado
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
obj_qtd = [3 3 3 3 3];
num_obj2 = sum(obj_qtd); %quantidade de material

%restri??o de ordem: linha = qual zona de descarga, coluna = qual ordem aquela
%zona pode ter
for i = 1:num_zonas %ordem generica (qualquer ordem)
    for j = 1:num_obj
        ordem(i,j,1:num_obj2)=1:num_obj2;
    end
end

%definir ordens diferentes das genericas
ordem(1,1,:) = [1 zeros(1,num_obj2-1)]; %primeiro obj tem q ser o primeiro na primeira zona
for i = 2:num_obj
    ordem(1,i,:) = [0 2:num_obj2];
end
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
demanda_zona = [2 2 2 2]; %nao colocar 0, ao inves, apagar zona
%demanda_full = 0; %torna-se 1 quando a demanda de todas as zonas ? atendida
obj_sobra = max(num_obj2 - sum(demanda_zona), 0); %numero de objetos que sobrar?o

Ncel = num_col * num_lin;

%Defini?ao dos vetores de probabilidades para i-esima a?ao de carga
%linhas: numero de a?oes de carga a serem realizadas
%colunas: qual objeto ser? capturado na i-esima a?ao de carga
for k = 1:num_transp
    for i = 1:num_obj2 % numero de a?oes de carga X numero de zonas de carga
        prob1(i,:,k) = (1/num_obj)*ones(1,num_obj);
    end
end
%Defini?ao dos vetores de probabilidades para descarga de cada pe?a
%linhas: objeto que foi capturado
%colunas: zona de descarga onde ser? descarregado o objeto i
for k = 1:num_transp
    for i = 1:num_obj2 % numero de objetos X numero de zonas de descarga
        prob2(i,:,k) = (1/num_zonas)*ones(1,num_zonas);
    end
end

%Defini?ao dos vetores de probabilidades para quantidade de carga
for k = 1:num_transp
    carga(k) = 2; %numero de objetos que podem ser carregados pelo drone
    for i = 1:num_obj2
        if(num_obj2-i>=carga(k)-1)
            p_carga(i,:,k) = (1/carga(k))*ones(1,carga(k));
        else
            p_carga(i,1:(num_obj2-i+1),k) = (1/(num_obj2-i+1))*ones(1,(num_obj2-i+1));
        end
    end
end

k=1;
dist_min = 999;
dist_media = 999;
dist = 0;

%cond1 = sum(sum(max(prob1,[],2)<prob_conv)>obj_sobra);
%cond2 = sum(sum(max(prob2,[],2)<prob_conv)>obj_sobra);
cond1 = sum(sum(max(prob1,[],2)<prob_conv))>obj_sobra+(num_transp-1)*num_obj2;
cond2 = sum(sum(max(prob2,[],2)<prob_conv))>obj_sobra+(num_transp-1)*num_obj2;

log_count = 0;
A_count = 0;
while (k <= NIterMax && ((cond1 || cond2) || (round(dist_media)*1e4/1e4)~=(round(dist)*1e4/1e4)))
    t_iter1 = now;
    disp("iter "+k)
    %title("iter "+k)
    %probab acumulada das a??es de carga
    for n = 1:num_transp
        prob_sum1(1,1,n)=prob1(1,1,n);
        for j=2:num_obj
            prob_sum1(1,j,n)=prob_sum1(1,j-1,n)+prob1(1,j,n);
        end
    end
    %probab acumulada das a??es de descarga
    for n = 1:num_transp
        for i=1:num_obj2
            prob_sum2(i,1,n)=prob2(i,1,n);
            for j=2:num_zonas
                prob_sum2(i,j,n)=prob_sum2(i,j-1,n)+prob2(i,j,n);
            end
        end
    end
    %probab acumulada da quantidade de carga
    for n = 1:num_transp
        for i=1:num_obj2
            p_carga_sum(i,1,n)=p_carga(i,1,n);
            for j=2:carga
                p_carga_sum(i,j,n)=p_carga_sum(i,j-1,n)+p_carga(i,j,n);
            end
        end
    end
    
    for n = 1:num_transp
        pos1(n)=no_inic(n);
        [i1(n),j1(n)]=pos_to_ij(pos1(n));
        dist(n) = 0;
        mov1(n,:) = zeros(1,num_obj2);
        mov2(n,:) = zeros(1,num_obj2);
        num_carga(n,:) = zeros(1,num_obj2);
        k2(n) = 1; %iterador de a??es de carga
        k3(n) = 1; %iterador de a??es de descarga
        carga_iter(n) = 0;
        teta(n) = 0;
%         em_carga(n) = 0;
        finished(n) = 0; %indicador se transp terminou atividades
    end
    prob_temp1 = prob1;
    prob_temp2 = prob2;
    carga_zona = zeros(1,num_zonas);
    count_descarga = 0;
    count_steps = 0;
    
    e_ordem = 0;
    e_zona = 0;
    obj_qtd_temp = obj_qtd;
    num_obj_decididos = 0;

    %% Decis?o de quantos objetos ser?o capturados por vez 
    for n = 1:num_transp
        max_carga(n)=decidir_qtd_carga(n);
    end
    %% Decis?o de quais objetos ser?o capturados
    for n = 1:num_transp
        pos2(n)=decidir_carga(n);
        em_carga(n) = 1;
    end

    %% Movimenta??es dos transportadores
    %while(sum(obj_qtd_temp) > obj_sobra) %condi??o antiga
    count2 = zeros(1,num_transp);
    while(count_descarga < num_obj2-obj_sobra || sum(finished)<2*num_transp)
        %count_descarga
        %title("iter "+k+", mov "+k2)
        for n = 1:num_transp
            if(finished(n)<2)
                if(pos1(n)==pos2(n)) %transportador chegou ao destino
                    if(finished(n)==1)
                        if(pos1(n)==no_inic(n))
                            if(debug) disp("transp "+n+" terminou e chegou no inicio");end
                            finished(n)=2;
                        end
                    elseif((carga_iter(n) < max_carga(n)) && em_carga(n)==1)
                        if(debug) disp(n+" chegou para carregar"); end
                        carga_iter(n) = carga_iter(n) + 1;
                        if(carga_iter(n)==max_carga(n))
                            pos2(n) = decidir_descarga(n);
                            em_carga(n) = 0;
                        else
                            pos2(n) = decidir_carga(n);
                        end
                    else
                        if(debug)  disp(n+" chegou para descarregar"); end
                        count_descarga = count_descarga + 1;
                        carga_iter(n) = carga_iter(n) - 1;
                        if(carga_iter(n)>0)
                            pos2(n) = decidir_descarga(n);
                        elseif(count_descarga ~= num_obj2-obj_sobra)
                            max_carga(n)=decidir_qtd_carga(n);
                            pos2(n) = decidir_carga(n);
                            em_carga(n) = 1;
                        else
                            pos2(n) = no_inic(n);
                            finished(n)=1;
                            if(debug) disp("transp "+n+" terminou, indo para no_inic"); end
                        end
                    end
                end

                if(e_ordem>0) %erro
                    if(debug) disp("erro ordem na iter "+k);end
                    break
                end
                
                %[i1(n),j1(n)]=pos_to_ij(pos1(n));
                [i2(n),j2(n)]=pos_to_ij(pos2(n));

                if(pos1(n)==pos2(n))
                    if(debug) disp("transp "+n+" parado"); end
                    gsuc(n) = 0; count(n) = 0; waypoints = [i1(n) j1(n)];
                else
                      %considerar outros drones como obstaculos (comentado,
                      %feito em rota_multi.m)
%                     obs = [];
%                     obs_count = 0;
%                     for m = 1:num_transp
%                         if(m~=n)
%                             obs_count = obs_count + 1;
%                             obs(obs_count,:) = [i1(m) j1(m)];
%                         end
%                     end
                    
                    %disp(n+" "+pos1(n)+" "+pos2(n)+" "+obs)
                    %log de A*
                    log_found = 0;
                    if(use_log)
                        for i=1:length(log(:,1))
                            if([i1(n) j1(n) i2(n) j2(n) teta(n)] == [log(i,1) log(i,2) log(i,3) log(i,4) log(i,5)])
                                teta(n) = log(i,6);
                                gsuc(n) = log(i,7);
                                waypoints(1) = log(i,8);
                                waypoints(2) = log(i,9);
                                log_found = 1;
                                log_count = log_count + 1;
                                k_log(end+1) = k;
                                break
                            end
                        end
                    end
                    if(log_found==0)
                        aux = teta(n);
                        [teta(n), gsuc(n), count(n), ~, waypoints] = Astar_multi([i1(n) j1(n)], [i2(n) j2(n)], teta(n), desenhar, cor(n), obs);       
                        A_count = A_count+1;
                        if(use_log)
                            log(end+1,:)=[i1(n) j1(n) i2(n) j2(n) aux teta(n) gsuc(n) waypoints(1) waypoints(2)];
                        end
                    end
                end
                if(debug_mov) disp("transp "+n+": indo de "+pos1(n)+" para "+pos2(n)); end
                dist(n) = dist(n) + gsuc(n);

                pos1(n) = (waypoints(1)-1)*num_col + waypoints(2);
                [i1(n),j1(n)]=pos_to_ij(pos1(n));
                if(debug_mov) disp("transp "+n+" chegou em "+pos1(n)); disp(" "); end
                if(pausar)
                    if(tempo_pause)
                        pause(tempo_pause);
                    else
                        pause;
                    end
                end
                %if(desenhar && (count(n)>0))
                %    children = get(gca, 'children');
                %    delete(children(1:count(n)));
                %end
            else
                if(debug)
                    disp("transp "+n+" nao tem atividades");
                    count2(n) = count2(n)+1;
                    if(min(count2)==20)
                        disp("erro: loop infinito")
                        return
                    end
                end
            end
        end
        count_steps = count_steps + 1;
        if(e_ordem>0)
            break
        end
    end
    
    
    %% fim das itera??es, inicio do aprendizado 
    % Armazena a distancia total da itera??o
    if(debug) disp("terminada itera??o "+k); end

    %completar variaveis com zeros
    aux = zeros(num_transp,num_obj2);
    aux(1:num_transp,1:length(mov1(1,:))) = mov1;
    mov1 = aux;
    
    aux = zeros(num_transp,num_obj2);
    aux(1:num_transp,1:length(mov2(1,:))) = mov2;
    mov2 = aux;
    
    aux = zeros(num_transp,num_obj2);
    aux(1:num_transp,1:length(obj2(1,:))) = obj2;
    obj2 = aux;
    
    %aux = zeros(num_transp,num_obj2);
    %aux(1:num_transp,1:length(num_carga(1,:))) = mov1;
    %num_carga = aux;
    
    dist = sum(dist);
    dist_hist(k) = dist;
    mov1_hist(k,:,:) = mov1;
    mov2_hist(k,:,:) = mov2;
    obj2_hist(k,:,:) = obj2;
    num_carga_hist(k,:,:) = num_carga;
    %carga_hist(k,:,:) = num_carga;
    if((dist < dist_min) && dist~=0)
        dist_min = dist;
    end
    
    best(k)=dist_min;
    count_steps_hist(k) = count_steps;
    %vetor de itera?oes
    x(k)=k;

    %Calculo do refor?o
    if(~e_ordem && ~e_zona)
        if k >= NStepMax+length(erros)
            %numero de itera?oes para compor a media
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
            %Atualiza??o das prob de carga
            for n = 1:num_transp
                for i=1:num_obj2
                    if(mov1(n,i)~=0)
                        if(metodo_atualizacao)
                            prob1(i,mov1(n,i),n)=(1+neta*reforco(k))*prob1(i,mov1(n,i),n);
                        else
                            prob1(i,mov1(n,i),n)=prob1(i,mov1(n,i),n) + neta*reforco(k)*(1-prob1(i,mov1(n,i),n));
                        end
                        if prob1(i,mov1(n,i),n) < 0
                            prob1(i,mov1(n,i),n)=0;
                        end
                        if(metodo_normalizacao)
                            prob1(i,:,n)=prob1(i,:,n)/sum(prob1(i,:,n)); % normaliza??o
                        else
                            for j=1:num_obj
                                if(j~=mov1(n,i))
                                    prob1(i,j,n)=prob1(i,j,n) - neta*reforco(k)*prob1(i,j,n);
                                end
                            end
                        end
                    end
                end
            end
            %Atualiza??o das prob de descarga
            for n = 1:num_transp
                z=0;
                for i=1:num_obj2
                    if(mov2(n,i)==0)
                        z = z + 1;
                    else
                        if(metodo_atualizacao)
                            prob2(obj2(n,(i-z):i),mov2(n,i),n)=(1+neta*reforco(k))*prob2(obj2(n,(i-z):i),mov2(n,i),n);
                        else
                            prob2(obj2(n,(i-z):i),mov2(n,i),n)=prob2(obj2(n,(i-z):i),mov2(n,i),n)+neta*reforco(k)*(1-prob2(obj2(n,(i-z):i),mov2(n,i),n));
                        end
                        for j = (i-z):i
                            if prob2(obj2(n,j),mov2(n,i),n) < 0
                               prob2(obj2(n,j),mov2(n,i),n)=0;
                            end
                            %normaliza??o
                            if(metodo_normalizacao)
                                prob2(obj2(n,j),:,n)=prob2(obj2(n,j),:,n)/sum(prob2(obj2(n,j),:,n));
                            else
                                for m=1:num_zonas
                                    if(m~=mov2(n,i))
                                        prob2(obj2(n,j),m,n)=prob2(obj2(n,j),m,n) - neta*reforco(k)*prob2(obj2(n,j),m,n);
                                    end
                                end
                            end
                        end
                        z=0;
                    end
                end
            end
            %aprendizado de p_carga (num_carga)
            for n = 1:num_transp
                if(carga(n)>1)
                    for i=1:length(num_carga(1,:))
                        if(num_carga(n,i)~=0)
                            p_carga(i,num_carga(n,i),n)=(1+neta*reforco(k))*p_carga(i,num_carga(n,i),n);

                            if p_carga(i,num_carga(n,i),n) < 0
                               p_carga(i,num_carga(n,i),n) = 0;
                            end
                            p_carga(i,:,n)=p_carga(i,:,n)/sum(p_carga(i,:,n)); % normaliza??o
                        end
                    end
                end
            end
        else
            reforco(k) = 0;
        end
    else
        erros = cat(2, erros_ordem, erros_zona);
        reforco(k) = 0; %old: -8
%         if(e_ordem)
%             prob1(k2,mov1(k2))=(1+neta*reforco(k))*prob1(k2,mov1(k2));
%             if prob1(k2,mov1(k2)) < 0
%                prob1(k2,mov1(k2))=0;
%             end
%             prob1(k2,:)=prob1(k2,:)/sum(prob1(k2,:)); % normaliza??o
%         end
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
                prob2(obj2(index(i)),:)=prob2(obj2(index(i)),:)/sum(prob2(obj2(index(i)),:)); % normaliza??o
            end
            %diminui prob de carregar mais de 1 objeto
%             if(carga_iter>1)
%                 i=k3-carga_iter+1;
%                 p_carga(i,num_carga(i))=(1+neta*reforco(k))*p_carga(i,num_carga(i));
%                 if p_carga(i,num_carga(i)) < 0
%                    p_carga(i,num_carga(i)) = 0;
%                 end
%                 p_carga(i,:)=p_carga(i,:)/sum(p_carga(i,:)); % normaliza??o
%             end
        end
    end
%     prob2
    %prob1(1:ceil(sum(demanda_zona)/num_transp),:,:)
%     if(carga>1)
%         p_carga(1:ceil(sum(demanda_zona)/num_transp),:,:)
%     end
    
    for n=1:num_transp
        aut1(n) = max(max(prob1(:,:,n),[],2));
        aut2(n) = max(max(prob2(:,:,n),[],2));
        Tcgmin(n,k) = min(aut1(n),aut2(n));
        Tcgmax(n,k) = max(aut1(n),aut2(n));
        Tcgmed(n,k) = (aut1(n)+aut2(n))/2;
    end
    clear p q;
    %y(k,1)=mov(1);
    %y(k,2)=mov(2);
    %y(k,3)=mov(3);
    oo = sum(sum(max(prob1,[],2)<prob_conv));
    pp = sum(sum(max(prob2,[],2)<prob_conv));
    disp(" "+oo+" "+pp)
    cond1 = sum(sum(max(prob1,[],2)<prob_conv))>obj_sobra+(num_transp-1)*num_obj2;
    cond2 = sum(sum(max(prob2,[],2)<prob_conv))>obj_sobra+(num_transp-1)*num_obj2;
    t_iter2 = now;
    t_iter(k) = (t_iter2-t_iter1)*24*60*60;
    k = k + 1;
    
%     z1(k-1,:) = prob2(1,:,2);
%     for n = 1:num_transp
%         z1(k-1,:,n)=max(prob1(1,:,n));
%         z2(k-1,:,n)=max(prob1(2,:,n));
%         z3(k-1,:,n)=max(prob1(3,:,n));
%     end
end

num_carga_hist_last = num_carga_hist(end-5:end,:,1:5)

% figure(1)
% for i = 1:8
%     subplot(4,2,i)
%     plot(x, obj2_hist(:,1,i), x, obj2_hist(:,2,i),x,obj2_hist(:,3,i))
% end

% plot(x,z1(1:end,:))
% hold on
% plot(x,z3(2:end,:))

t2=now;
elapsed_days1 = t2 - t1;
elasped_second1 = elapsed_days1 *24*60*60;
disp("tempo total: "+elasped_second1)
disp("n_iter: "+k)
disp("tempo/iter: "+(elasped_second1/k))

% plotMap
prob1(1:ceil(sum(demanda_zona)/num_transp)+2,:,:)
% prob2
% if(carga>1)
%     p_carga
% end

%subplot(3,1,1)
%plot(x,y(:,1))
%subplot(3,1,2)
%plot(x,y(:,2))
%subplot(3,1,3)
%plot(x,y(:,3))
%figure(2)

f1 = figure(1);
figure(f1)
a=plot(x(best~=999),best(best~=999),'g');
hold on
b=plot(setdiff(x,erros),dist_hist(dist_hist~=0 & dist_hist~=999));
xlabel("n?mero de itera??es")
ylabel("dist?ncia total percorrida")
title("dist?ncia total percorrida em cada itera??o")
legend([b,a],"dist?ncia total percorrida","dist?ncia m?nima encontrada")

f2 = figure(2);
figure(f2)
b=plot(setdiff(x,erros),count_steps_hist(count_steps_hist~=1));

figure(3)
plot(x,reforco)

% figure(2)
% hold on
% c=plot(x,t_iter);
% title("A count: "+A_count+", log count: "+log_count)
% grid on
% if(length(x)>250)
%     %title("media das ultimas 200 iter: "+sum(t_iter(end-200:end))/200)
%     if(use_log)
%         legend(c,"media das ultimas 200 iter COM log: "+sum(t_iter(end-200:end))/200)
%     else
%         legend(c,"media das ultimas 200 iter SEM log: "+sum(t_iter(end-200:end))/200)
%     end
% end

% for n=1:num_transp
% figure(3+n)
% plot(x, Tcgmin(n,:),'b')
% hold on
% plot(x, Tcgmed(n,:),'r')
% plot(x, Tcgmax(n,:),'k')
% legend("Tcgmin","Tcgmed","Tcgmax")
% title("Taxa de convergencia geral (Tcg), transp "+n) %atualizacao="+metodo_atualizacao+", normalizacao="+metodo_normalizacao)
% grid
% end

[dist,waypoints2]=rota_multi(prob1,prob2,p_carga, obj_qtd,demanda_zona);

function max_carga = decidir_qtd_carga(n)
    global p_carga_sum k2 carga obj_sobra num_obj_decididos num_obj2 num_carga debug
    %if(length(num_carga(n))<k2 || (length(num_carga(n))==1 && num_carga(n)==0))
    p=rand;
    q=sum(p<=p_carga_sum(k2(n),:,n));
    num_carga(n,k2(n))=carga(n)-q+1;
    
    %caso numero de cargas necessarias para finalizar tarefa seja menor que
    %num_carga selecionado, diminuir num_carga
    %disp("obj_qtd_temp: "+obj_qtd_temp)
    if(num_carga(n,k2(n))>num_obj2-num_obj_decididos-obj_sobra)
        num_carga(n,k2(n)) = num_obj2-num_obj_decididos-obj_sobra;
    end
    num_obj_decididos = num_obj_decididos + num_carga(n,k2(n));
    
    %caso todas as zonas aceitem apenas x cargas, num_carga nao pode ser
    %maior que x
    %{
    if(num_carga(n,k2(n))>max(demanda_zona-carga_zona))
        num_carga(n,k2(n)) = max(demanda_zona-carga_zona);
    end
    %}
    if(num_carga(n,k2(n))>1)
        num_carga(n,k2(n)+1:k2(n)+num_carga(n,k2(n))-1)=0; %se iter k2 pegou x objetos, iter k2+1:k2+x-1 pega 0
    end
    
    if(debug) disp(n+" decidindo qtd_carga: "+num_carga(n,k2(n))); end
    %end
    %if(num_carga(n,k2)~=0)
    max_carga = num_carga(n,k2(n));
    %end
end

function pos = decidir_carga(n)
    global k2 prob_sum1 prob_temp1 obj_qtd_temp mov1 num_obj num_obj2 obj num_transp
    global finished no_inic debug obj_sobra obj_qtd obj2 num_obj_decididos carga_iter
    global num_carga
    p=rand;
    
    if(debug) disp("transp "+n+", k2= "+k2(n)); end
    %prob_temp1(:,:,n)
    %prob_sum1(:,:,n)
    q=sum(p<=prob_sum1(k2(n),:,n));
    mov1(n,k2(n))=num_obj-q+1; % no. do movimento: 1 a Nmov
    if(q==0)
        mov1(n,k2(n))=0;
        if(debug) disp(n+" nao encontrou carga, terminou, indo para no_inic"); end 
        finished(n) = 1;
        pos=no_inic(n); %caso nao existam mais obj a serem pegados
    elseif(num_obj2-num_obj_decididos==obj_sobra && num_carga(n,k2(n))==0 && carga_iter(n)==0)
        mov1(n,k2(n))=0;
        if(debug) disp(n+" demanda atingida, terminou, indo para no_inic"); end 
        finished(n) = 1;
        pos=no_inic(n); %caso nao existam mais obj a serem pegados
    else
        pos=obj(mov1(n,k2(n))); % nova posi??o
        if(debug) disp("transp "+n+": carga em "+pos); end
        %erro de ordem -> passado para decidir_descarga
        %{
        if(~ismember(mov1(n,k2),ordem(k2,:)))
            e_ordem = 1; 
            erros_ordem(end+1) = k; %salva itera?ao que deu erro
            if(k2<num_obj2)
                mov1(k2+1:num_obj2) = 0;
                mov2(k2+1:num_obj2) = 0;
                num_carga(k2+1:num_obj2) = 0;
                obj2(k2+1:num_obj2) = 0;
            end
            dist = 0;
            break
        end
        %}         
        obj_qtd_temp(mov1(n,k2(n)))=obj_qtd_temp(mov1(n,k2(n)))-1;
        %disp("sum obj_qtd_temp: "+sum(obj_qtd_temp));
        obj2(n,k2(n)) = sum(obj_qtd(1:mov1(n,k2(n))))-obj_qtd_temp(mov1(n,k2(n)));
%         disp("obj_qtd_temp: "+sum(obj_qtd_temp));
        if(obj_qtd_temp(mov1(n,k2(n)))<1)
            mov = mov1(n,k2(n));
            %prob_temp1(k2(n)+1:end,mov,n) = 0; not necessary anymore
            for m = 1:num_transp
                same_drone = (m==n); %se mesmo drone, atualiza iter k2+1 pra frente, senao, iter k2 pra frente
                if ((k2(n) >= num_obj2) && same_drone)
                    if(debug) disp("movimento final de "+n); end
                else
                    prob_temp1(k2(m)+same_drone:end,mov,m) = 0; %zera probabilidade da carga que ja foi capturada
                end
            end
        end
        %disp("apos atualiza?ao:")
        k2(n) = k2(n) + 1;
        for m=1:num_transp
            if(sum(prob_temp1(k2(m),:,m))~=0)
                prob_temp1(k2(m),:,m)=prob_temp1(k2(m),:,m)/sum(prob_temp1(k2(m),:,m)); % normaliza??o
            end
            prob_sum1(k2(m),1,m)=prob_temp1(k2(m),1,m);
            for j=2:num_obj
                prob_sum1(k2(m),j,m)=prob_sum1(k2(m),j-1,m)+prob_temp1(k2(m),j,m);
            end
        end
        %prob_temp1(:,:,1)
        %prob_temp1(:,:,2)
        %prob_sum1(:,:,1)
        %prob_sum1(:,:,2)
    end
end

function pos = decidir_descarga(n)
    global k3 prob_sum2 prob_temp2 num_obj2 num_transp debug
    global num_zonas mov2 carga_zona demanda_zona no_final obj2 %carga_iter
    global ordem mov1 e_ordem erros_ordem num_carga k2 dist k count_steps %variaveis necessarias para erro_ordem
    %global obj_qtd obj_qtd_temp mov1 k k2
    %k3(n) = k2(n) - 1;
    %aux = 0;
    p=rand;
    
    %caso esteja com mais de 1 obj, escolher zona de descarga pela
    %media de probabilidades dos objetos com as zonas
    %{
    prob_mix = zeros(1,num_zonas);
    for i=(k3(n)-carga_iter(n)+1):k3(n)
        if(carga_iter(n)>1 && i~=k3(n))
            j = i;
            while(mov1(n,j)==mov1(n,j+1))
                aux = aux + 1; j = j + 1;
                if(j==k3(n)) break; end
            end
        else
            aux=0;
        end
        obj2(i) = sum(obj_qtd(1:mov1(i)))-obj_qtd_temp(mov1(i))-aux;
        prob_mix = prob_mix + prob_sum2(n,obj2(i),:);
    end
    prob_mix = prob_mix/carga_iter
    q=sum(p<=prob_mix);
    %}
    
    %passado para decidir_carga
    %obj2(n,k3(n)) = sum(obj_qtd(1:mov1(n,k2(n)-1)))-obj_qtd_temp(mov1(n,k2(n)-1));
    
    %caso nem todas as zonas tenham espa?o suficiente para carga_iter atual,
    %escolher apenas dentre zonas que aceitam carga_iter atual
    %{
    teste_zona_carga = carga_iter(n)<=(demanda_zona-carga_zona);
    if(sum(teste_zona_carga)~=num_zonas)
        prob_temp2_aux = prob_temp2(obj2(n,k3(n)),:,n).*teste_zona_carga; %remover zonas que nao aceitam
        prob_temp2_aux = prob_temp2_aux/sum(prob_temp2_aux); % normaliza??o
        prob_sum2_aux(1)=prob_temp2_aux(1);
        for j=2:num_zonas
            prob_sum2_aux(j)=prob_sum2_aux(j-1)+prob_temp2_aux(j);
        end
        q=sum(p<=prob_sum2_aux);
    else
        q=sum(p<=prob_sum2(obj2(n,k3(n)),:,n));
    end
    %}
    q=sum(p<=prob_sum2(obj2(n,k3(n)),:,n));
    mov2(n,k3(n))=num_zonas-q+1; % no. do movimento: 1 a num_zonas
    pos=no_final(mov2(n,k3(n))); % nova posi??o
    if(debug) disp("transp "+n+": descarga em "+pos); end
    
    %erro de ordem
    %zona de descarga escolhida: mov2(n,k3(n))
    %zona de carga a ser descarregada: mov1(n,k3(n))
    %ordem que o obj est? sendo descarregado: carga_zona(mov2(n,k3(n)))+1
    %ordem a ser checada: ismember(carga_zona(mov2(n,k3(n)))+1,ordem(mov2(n,k3(n)),mov1(n,k3(n)),:))
    if(~ismember(carga_zona(mov2(n,k3(n)))+1,ordem(mov2(n,k3(n)),mov1(n,k3(n)),:)))
        if(debug) disp("erro ordem");end
        e_ordem = n; 
        erros_ordem(end+1) = k; %salva itera?ao que deu erro
        if(k2(n)<num_obj2)
            mov1(n,k2(n)+1:num_obj2) = 0;
            mov2(n,k2(n)+1:num_obj2) = 0;
            num_carga(n,k2(n)+1:num_obj2) = 0;
            obj2(n,k2(n)+1:num_obj2) = 0;
            dist(:)=0;
            count_steps = 0;
        end
    else
        carga_zona(mov2(n,k3(n))) = carga_zona(mov2(n,k3(n))) + 1;
        if(carga_zona(mov2(n,k3(n)))>=demanda_zona(mov2(n,k3(n))))
            for m = 1:num_transp
                prob_temp2(:,mov2(n,k3(n)),m) = 0; %zona nao aceita mais cargas
            end
            %checa se os objetos foram colocados na zona correta
            %{
            if(~prod(ismember(mov1(k3-carga_iter+1:k3),tipo_zona(mov2(k3),:))))
                %disp("mov1: "+mov1(k3-carga_iter+1:k3))
                %disp("mov2: "+mov2(k3))
                e_zona = 1; %erro de zona
                erros_zona(end+1) = k; %salva itera?ao que deu erro
                if(k3<num_obj2)
                    mov1(k2:num_obj2) = 0;
                    mov2(k2:num_obj2) = 0;
                    num_carga(k2:num_obj2) = 0;
                    obj2(k2:num_obj2) = 0;
                end
                dist = 0;
                break
            end
    %}
            %caso todas as demandas sejam atendidas
            %{
            if(sum(prob_temp2(i,:,n))==0) 
                %demanda_full = 1;
                mov1(k2:num_obj2) = 0;
                mov2(k2:num_obj2) = 0;
                num_carga(k2:num_obj2) = 0;
                obj2(k2:num_obj2) = 0;
                break
            end
            %}
        end
        k3(n) = k3(n) + 1;
        for m = 1:num_transp
            for i=1:num_obj2
                if(sum(prob_temp2(i,:,m))~=0)
                    prob_temp2(i,:,m)=prob_temp2(i,:,m)/sum(prob_temp2(i,:,m)); % normaliza??o
                end
                prob_sum2(i,1,m)=prob_temp2(i,1,m);
                for j=2:num_zonas
                    prob_sum2(i,j,m)=prob_sum2(i,j-1,m)+prob_temp2(i,j,m);
                end
            end
        end
    end
end

function [i1, j1] = pos_to_ij(pos)
    global num_col
    i1=fix(pos/num_col)+1;
    j1=mod(pos,num_col);
    if j1==0
        j1 = num_col; i1 = i1-1;
    end
end