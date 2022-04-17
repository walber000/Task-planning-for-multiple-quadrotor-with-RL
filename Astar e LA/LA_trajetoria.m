% Learning automata para decisão de melhor sequência de captura de objetos
% em diferentes posições.
% Adaptado de rl_lab.m (LA para soluçao de labirinto) do prof. Cairo.
% Walber Lima P. Junior, 05/09/19

clear all; close all;
desenhar = 0; pausar = 0; tempo_pause = 0.7;
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

%rectangle('Position',[j-tc/2,i-tc/2,tc,tc],'Curvature',[1 1])
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

% Definicao dos parametros do vetor de recompensas LAB_REC
% O vetor LAB_REC define a recompensa obtida em cada passo quando a iteração termina em uma
% determinada celula
%rec_neg=-2; rec_pos=2; % Recompensa/Castigo por terminar iteração em tempo pior ou melhor que a media

% Definicao dos parametros do metodo Learning Automata
NIterMax=1000; % no. de iterações total de aprendizado
NStepMax=15; %no. de passos/iteraçoes para começar a atualizar probabilidades
prob_conv = 0.95; %probab para considerar convergencia alcançada
neta=0.2;     % taxa de aprendizado

% PASSO 3: Definicao inicial da matriz LAB_PROP
%rand('state',0); % Comentar esta linha para inicialização diferente em cada execução do programa

% LAB_PROP indica a probabilidade calculada para a execucao de um certo movimento
% LAB_PROP(i,j) indica a probabilidade de execucao do movimento com destino
% 'a celula LAB_POS(i,j) a partir da celula i.
% A soma de cada linha de LAB_PROP deve ser sempre igual a 1.
Ncel = num_col * num_lin;
Nmov = num_obj;



%Definiçao dos vetores de probabilidades para i-esima açao de carga
for i = 1:Nmov
    prob(i,:) = (1/Nmov)*ones(1,Nmov);
end

% PASSO 5: Loop de aprendizado usando o metodo Bootstrap Learning
k=1;
while (k <= NIterMax && sum(max(prob,[],2)<prob_conv)~=0 )
    disp("iter "+k)
    prob_sum(1,1)=prob(1,1);
    for j=2:Nmov;
        prob_sum(1,j)=prob_sum(1,j-1)+prob(1,j);
    end;
    pos(1)=no_inic;
    prob_temp = prob;
    dist = 0;
    k2 = 1;
    while(k2 <= Nmov)  
        %Carga
        title("iter "+k+", mov "+k2)
        %disp("iter 1")
        p=rand;
        q=sum(p<=prob_sum(k2,:));
        mov(k2)=Nmov-q+1; % no. do movimento: 1 a Nmov
        %pos(k2+1)=1
        %obj(mov(k2))
        pos(k2+1)=obj(mov(k2)); % nova posição
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
        if ((k2>1 || k>1) && desenhar)
            % Apagar linha feita pelo A*
            children = get(gca, 'children');
            delete(children(1:count));
        end
        [gsuc, count] = Astar([i1 j1], [i2 j2], desenhar, 1);
        dist = dist + gsuc;

        k2 = k2 + 1;
        if k2 <= Nmov
            prob_temp(k2:end,mov(k2-1)) = 0; %zera probabilidade da carga que ja foi capturada
            prob_temp(k2,:)=prob_temp(k2,:)/sum(prob_temp(k2,:)); % normalização
            prob_sum(k2,1)=prob_temp(k2,1);
            for j=2:Nmov;
                prob_sum(k2,j)=prob_sum(k2,j-1)+prob_temp(k2,j);
            end;
        end
        if(pausar)
            if(tempo_pause)
                pause(tempo_pause);
            else
                pause;
            end
        end
        % Descarga:
        % Apagar linha feita pelo A*
        if(desenhar)
            children = get(gca, 'children');
            delete(children(1:count));
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
    end
    % Armazena a distancia total da iteração
    dist_hist(k,:) = dist;
    mov_hist(k,:) = mov;

    if k == 1
        dist_min = dist;
        %custo
    else
        if(dist < dist_min)
            dist_min = dist;
        end
    end
    
    if k >= NStepMax
        %aprendizado
        dist_media = sum(dist_hist(end-NStepMax+1:end))/NStepMax;
        reforco(k) = sign((dist_media - dist)/(dist_media - dist_min));
        if dist_media == dist_min
            reforco(k) = 1;
        end
        %if (reforco(k) < 0) reforco(k) = 0; end
        for i=1:Nmov;
            prob(i,mov(i))=(1+neta*reforco(k))*prob(i,mov(i));
            if prob(i,mov(i)) < 0
               prob(i,mov(i))=0;
            end
            prob(i,:)=prob(i,:)/sum(prob(i,:)); % normalização
        end;
    end
    prob
    x(k)=k;
    y(k,1)=mov(1);
    y(k,2)=mov(2);
    y(k,3)=mov(3);
    k = k + 1;
    clear p q;
end
figure(1)
subplot(3,1,1)
plot(x,y(:,1))
subplot(3,1,2)
plot(x,y(:,2))
subplot(3,1,3)
plot(x,y(:,3))
figure(2)
plot(x,reforco)

    %{
    % LEARNING
    if mov_hist_len(iter) > 1;
        %disp(['Learning at iteration ' num2str(iter)]);
        feed_rec=LAB_REC(pos(end));
        if (mov_hist_len(iter) >= NStepMax) && (LAB_REC(pos(end)) == rec_mp);
            feed_rec=rec_mp;
        end;
        rec_hist(iter)=feed_rec;
        for k=mov_hist_len(iter)-1:-1:1;
            j=mov(k);
            LAB_PROP(pos(k),j)=(1+neta*feed_rec)*LAB_PROP(pos(k),j);
            if LAB_PROP(pos(k),j) < 0; LAB_PROP(pos(k),j)=0; end;
            LAB_PROP(pos(k),:)=LAB_PROP(pos(k),:)/sum(LAB_PROP(pos(k),:)); % normalização
        end;
        fprintf('\n Iter. %3d: Rec = %4.1f, Mov = %2d /',iter,feed_rec,length(mov_hist{iter})-1);
        % fprintf(' %2d',mov_hist{iter});
        for k=1:mov_hist_len(iter)-1;
            j=mov(k);
            fprintf(' %d %s',pos(k),str1(j));
        end;
        fprintf(' %d',pos(end));
    end;
    
    % Mostrar o status da aprendizagem armazenado na matriz LAB_PROP
    if iter >= 1; % plotar o movimento mais provável em cada célula (em caso de empate é indicado
        % apenas o 1o. movimento na ordem RULD)
        strTitle=sprintf('Iteração = %3d/%3d',iter,NIterMax);
        if iter >=1; strTitle=[strTitle sprintf(', Rec = %4.1f',feed_rec)]; end;
        title([strTitle strRec]); % título da figura 1
        % plotar o movimento mais provável nas células visitadas nesta iteração
        for k=1:length(pos)-1;
            ic=pos(k);
            delete(hp(ic)); delete(hp2(ic)); % apagar seta da iteração anterior
            [Prob_max,Mov_max]=max(LAB_PROP(ic,:));
            i=ceil(ic/Nc); j=ic-(i-1)*Nc;
            hp(ic)=plot(j-0.5,Nl-i+0.5,[str1(Mov_max) 'k']); % plotar a ponta da seta
            hp2(ic)=plot([j-0.5 j-0.5+dx(Mov_max)],[Nl-i+0.5 Nl-i+0.5+dy(Mov_max)],'k-');
        end;
        % atualizar o no. de visitas nas células visitadas nesta iteração
        for k=1:length(pos);
            ic=pos(k);
            delete(htextC(ic));
            i=ceil(ic/Nc); j=ic-(i-1)*Nc;
            htextC(ic)=text(j,Nl-i,num2str(LAB_Count(ic)));
            set(htextC(ic),'HorizontalAlignment','right');
            set(htextC(ic),'VerticalAlignment','baseline');
            set(htextC(ic),'FontSize',8);
        end;
        % plotar as linhas que indicam as células visitadas na iteração atual
        if iter > 1; delete(hp3); end;
        hp3=[];
        for k=2:length(pos);
            hi(1)=fix((pos(k-1)-0.1)/Nc)+1;
            hi(2)=fix((pos(k)  -0.1)/Nc)+1;
            hj(1)=pos(k-1)-(hi(1)-1)*Nc;
            hj(2)=pos(k)  -(hi(2)-1)*Nc;
            hp3(k-1)=plot([hj(1)-0.5 hj(2)-0.5],[Nl-hi(1)+0.5 Nl-hi(2)+0.5],'r-');
            set(hp3(k-1),'LineWidth',2);
        end;
    end;
    drawnow;
    clear h;
    
    if iter<=10; pause; end;
    
end;
delete(hp3);

% Cálculo e gráfico da recompensa média a cada Np iterações
fprintf('\n Média do no. de movimentos em cada iteração = %.2f',mean(mov_hist_len));
Np=30;
figure;
iend=length(rec_hist);
for iter=fix(NIterMax/Np):-1:1;
    rec_avg(iter)=mean(rec_hist(iend-Np+1:iend));
    iend=iend-Np;
end;
plot(rec_avg,'-*');
xlabel('No. do intervalo');
ylabel('');
title(['Recompensa Média a cada ' num2str(Np) ' iterações']);
grid;

% Cálculo e figura da diferença entre as 2 maiores probabilidades em cada célula
Ncores=100;
map=colormap(gray(Ncores)); map=map(Ncores:-1:1,:);
% Delta_LP=sort(LAB_PROP','descend')'; Delta_LP=Delta_LP(:,1)-Delta_LP(:,2);
for i=1:Nl*Nc; % cobre o caso onde as 2 maiores probabilidades da célula são iguais
    % e maiores do que as outras probabilidades
    Delta_LP2=unique(LAB_PROP(i,:));
    Delta_LP(i)=0; % caso onde todas as probabilidades da célula são iguais
    if length(Delta_LP2) > 1; Delta_LP(i)=Delta_LP2(end)-Delta_LP2(end-1); end;
end;
D_LP=round(100*Delta_LP); maxD=max(D_LP); minD=min(D_LP);
ind_D_LP=1+(D_LP-minD)*(Ncores-1)/(maxD-minD);
for i=1:Nl; Image_DP(i,:)=ind_D_LP((i-1)*Nc+(1:Nc)); end;
figure; image(Image_DP); colormap(map);
title('Diferença entre as 2 maiores probabilidades em cada célula');

fprintf('\n--> Diferença entre as 2 maiores probabilidades em cada célula:');
for i=1:Nl; fprintf('\n'); fprintf(' %3.2f',Delta_LP((i-1)*Nc+(1:Nc))); end;

% Cálculo e figura do no. de passagens por cada célula
D_LP=LAB_Count; maxD=max(D_LP); minD=min(D_LP);
ind_D_LP=1+(D_LP-minD)*(Ncores-1)/(maxD-minD);
for i=1:Nl; Image_DP(i,:)=ind_D_LP((i-1)*Nc+(1:Nc)); end;
figure; image(Image_DP); colormap(map);
D_LP=unique(D_LP); str2=sprintf(' [%d, %d, ..., %d]',D_LP(1),D_LP(2),D_LP(end));
title(['No. de passagens por cada célula' str2]);

fprintf('\n--> No. de passagens por cada célula:');
for i=1:Nl; fprintf('\n'); fprintf(' %3d',LAB_Count((i-1)*Nc+(1:Nc))); end;
figure(1);
fprintf('\n');
%}
% EOF
