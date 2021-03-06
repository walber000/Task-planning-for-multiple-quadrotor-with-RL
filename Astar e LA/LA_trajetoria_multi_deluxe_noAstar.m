% Learning automata para decis?o de melhor sequ?ncia de captura de objetos
% em diferentes posi??es.
% Adaptado de rl_lab.m (LA para solu?ao de labirinto) do prof. Cairo.
% Walber Lima P. Junior, 05/09/19

clear all;
clc; close all;

global p_carga_sum k2 k3 carga prob_sum1 prob_sum2 prob_temp1 obj_qtd_temp mov1 mov2 k
global num_col obj_qtd num_obj num_obj2 obj num_transp num_zonas no_final carga_zona num_obj_decididos
global demanda_zona pos1 finished no_inic carga_iter debug obj2 prob_temp2 obj_sobra num_carga
global ordem e_ordem erros_ordem count_steps dist e_tipo_qtd e_ordem_c
global tipo_qtd_zona tipo_qtd_zona_temp erros_ordem_c erros_tipo_qtd 
for zz = 0:9
rand('state',zz);
desenhar = 0; pausar = 0; tempo_pause = 0; debug = 0; debug_mov = 0; show_plots = 0;
if(desenhar) plotMap; end
t1=now;

%% Defini??o do mapa

file = fopen('activemap_multi.txt','r');
size_map = fscanf(file,'%d',[1 2]);
num_lin = size_map(1,1);
num_col = size_map(1,2);
map = fscanf(file,'%d',[num_col num_lin]);
map = map';
Ncel = num_col * num_lin;
obj_qtd = [1 2 2];


% load('map4.mat')
% num_lin = size(map,1);
% num_col = size(map,2);

% mapa 100x100
% map = zeros(100,100);
% num_lin = 100;
% num_col = 100;
% map(20,20) = 2;
% map(80,80) = 2;
% map(20,80) = 3;
% map(40,80) = 3;
% map(80,20) = 3;
% map(50,50) = 3;
% map(100,20) = 4;
% map(100,40) = 4;
% map(100,60) = 4;
% map(70,100) = 4;
% map(50,100) = 4;
% % map(50:100,30) = 1;
% % map(60,80:100) = 1;
% obj_qtd = [1 2 2 2];

% mapa 150x150
% map = zeros(150,150);
% num_lin = 150;
% num_col = 150;
% map(20,20) = 2;
% map(40,20) = 2;
% map(60,20) = 2;
% map(20,80) = 3;
% map(40,80) = 3;
% map(20,60) = 3;
% map(40,60) = 3;
% map(100,100) = 3;
% map(140,25) = 4;
% map(140,50) = 4;
% map(140,75) = 4;
% map(75,140) = 4;
% map(50,140) = 4;
% map(25,140) = 4;
% % map(1:50,70) = 1;
% % map(65,110:150) = 1;
% obj_qtd = [1 2 2 2 2];

% mapa 200x200
% map = zeros(200,200);
% num_lin = 200;
% num_col = 200;
% map(20,20) = 2;
% map(40,20) = 2;
% map(60,20) = 2;
% map(80,20) = 2;
% map(20,180) = 3;
% map(40,180) = 3;
% map(20,60) = 3;
% map(40,60) = 3;
% map(100,100) = 3;
% map(100,120) = 3;
% map(140,25) = 4;
% map(140,50) = 4;
% map(140,75) = 4;
% map(75,140) = 4;
% map(50,140) = 4;
% map(25,140) = 4;
% map(140,140) = 4;
% % map(1:50,70) = 1;
% % map(100:150,70) = 1;
% % map(65,110:150) = 1;
% obj_qtd = [1 2 2 2 2 2];

% mapa 250x250
% map = zeros(250,250);
% num_lin = 250;
% num_col = 250;
% map(20,20) = 2;
% map(40,20) = 2;
% map(60,20) = 2;
% map(180,20) = 2;
% map(200,20) = 2;
% map(20,180) = 3;
% map(40,180) = 3;
% map(20,60) = 3;
% map(40,60) = 3;
% map(100,100) = 3;
% map(100,120) = 3;
% map(200,200) = 3;
% map(240,25) = 4;
% map(240,50) = 4;
% map(240,75) = 4;
% map(75,240) = 4;
% map(50,240) = 4;
% map(25,240) = 4;
% map(140,140) = 4;
% map(200,140) = 4;
% % map(1:50,70) = 1;
% % map(100:150,70) = 1;
% % map(85,200:250) = 1;
% % map(65,110:150) = 1;
% obj_qtd = [1 2 2 2 2 2 2];

% mapa 300x300
% map = zeros(300,300);
% num_lin = 300;
% num_col = 300;
% map(20,20) = 2;
% map(40,20) = 2;
% map(60,20) = 2;
% map(180,20) = 2;
% map(200,20) = 2;
% map(220,20) = 2;
% map(20,180) = 3;
% map(40,180) = 3;
% map(20,60) = 3;
% map(40,60) = 3;
% map(100,100) = 3;
% map(100,120) = 3;
% map(200,200) = 3;
% map(220,240) = 3;
% map(240,25) = 4;
% map(240,50) = 4;
% map(240,75) = 4;
% map(75,240) = 4;
% map(50,240) = 4;
% map(25,240) = 4;
% map(140,140) = 4;
% map(200,140) = 4;
% map(180,180) = 4;
% % map(1:50,70) = 1;
% % map(100:150,70) = 1;
% % map(85,200:250) = 1;
% % map(65,110:150) = 1;
% obj_qtd = [1 2 2 2 2 2 2 2];

% mapa 350x350
% map = zeros(350,350);
% num_lin = 350;
% num_col = 350;
% map(20,20) = 2;
% map(40,20) = 2;
% map(60,20) = 2;
% map(180,20) = 2;
% map(200,20) = 2;
% map(300,80) = 2;
% map(300,120) = 2;
% map(20,180) = 3;
% map(40,180) = 3;
% map(20,60) = 3;
% map(40,60) = 3;
% map(100,100) = 3;
% map(100,120) = 3;
% map(200,200) = 3;
% map(220,200) = 3;
% map(240,200) = 3;
% map(240,25) = 4;
% map(240,50) = 4;
% map(240,75) = 4;
% map(75,240) = 4;
% map(50,240) = 4;
% map(25,240) = 4;
% map(140,300) = 4;
% map(200,300) = 4;
% map(180,300) = 4;
% map(220,300) = 4;
% % map(1:50,70) = 1;
% % map(100:150,70) = 1;
% % map(85,200:250) = 1;
% % map(65,110:150) = 1;
% % map(150:200,240) = 1;
% obj_qtd = [1 2 2 2 2 2 2 2 2];

% map = zeros(400,400);
% num_lin = 400;
% num_col = 400;
% map(20,20) = 2;
% map(40,20) = 2;
% map(60,20) = 2;
% map(180,20) = 2;
% map(200,20) = 2;
% map(300,80) = 2;
% map(300,120) = 2;
% map(300,160) = 2;
% map(20,180) = 3;
% map(40,180) = 3;
% map(20,60) = 3;
% map(40,60) = 3;
% map(100,100) = 3;
% map(100,120) = 3;
% map(200,200) = 3;
% map(220,200) = 3;
% map(240,200) = 3;
% map(300,300) = 3;
% map(240,25) = 4;
% map(240,50) = 4;
% map(240,75) = 4;
% map(75,240) = 4;
% map(50,240) = 4;
% map(25,240) = 4;
% map(140,300) = 4;
% map(200,300) = 4;
% map(180,300) = 4;
% map(220,300) = 4;
% map(320,360) = 4;
% map(1:50,70) = 1;
% map(100:150,70) = 1;
% map(85,200:250) = 1;
% map(65,110:150) = 1;
% map(150:200,240) = 1;
% obj_qtd = [1 2 2 2 2 2 2 2 2 2];

%mapa 450x450
map = zeros(450,450);
num_lin = 450;
num_col = 450;
map(20,20) = 2;
map(40,20) = 2;
map(60,20) = 2;
map(180,20) = 2;
map(200,20) = 2;
map(300,80) = 2;
map(300,120) = 2;
map(300,160) = 2;
map(300,200) = 2;
map(20,180) = 3;
map(40,180) = 3;
map(20,60) = 3;
map(40,60) = 3;
map(100,100) = 3;
map(100,120) = 3;
map(200,200) = 3;
map(220,200) = 3;
map(240,200) = 3;
map(300,300) = 3;
map(330,350) = 3;
map(240,25) = 4;
map(240,50) = 4;
map(240,75) = 4;
map(75,240) = 4;
map(50,240) = 4;
map(25,240) = 4;
map(140,300) = 4;
map(200,300) = 4;
map(180,300) = 4;
map(220,300) = 4;
map(400,360) = 4;
map(400,320) = 4;
% map(1:50,70) = 1;
% map(100:150,70) = 1;
% map(85,200:250) = 1;
% map(65,110:150) = 1;
% map(150:200,240) = 1;
% map(250:350,250) = 1;
obj_qtd = [1 2 2 2 2 2 2 2 2 2 2];


% mapa 500x500
% map = zeros(500,500);
% num_lin = 500;
% num_col = 500;
% map(20,20) = 2;
% map(40,20) = 2;
% map(60,20) = 2;
% map(180,20) = 2;
% map(200,20) = 2;
% map(300,80) = 2;
% map(300,120) = 2;
% map(300,150) = 2;
% map(200,450) = 2;
% map(150,450) = 2;
% map(20,180) = 3;
% map(40,180) = 3;
% map(20,60) = 3;
% map(40,60) = 3;
% map(100,100) = 3;
% map(100,120) = 3;
% map(200,200) = 3;
% map(220,200) = 3;
% map(240,200) = 3;
% map(300,300) = 3;
% map(330,350) = 3;
% map(330,450) = 3;
% map(240,25) = 4;
% map(240,50) = 4;
% map(240,75) = 4;
% map(75,240) = 4;
% map(50,240) = 4;
% map(25,240) = 4;
% map(140,300) = 4;
% map(200,300) = 4;
% map(180,300) = 4;
% map(220,300) = 4;
% map(400,360) = 4;
% map(400,320) = 4;
% map(400,400) = 4;
% % map(1:50,70) = 1;
% % map(100:150,70) = 1;
% % map(85,200:250) = 1;
% % map(65,110:150) = 1;
% % map(150:200,240) = 1;
% % map(250:350,250) = 1;
% obj_qtd = [1 2 2 2 2 2 2 2 2 2 2 2];

% map2rgb(map);
% return

num_obj = 0; %iterador para numero de objetos
num_zonas = 0; %iterador para numero de zonas de descarga
num_transp = 0; %numero de transportadores
log = [0 0 0 0 0 0 0]; %log de A*: [i1 j1 i2 j2 teta newteta gsuc wp1 wp2]
log_wp = {0};
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

%% Definicao dos parametros do metodo Learning Automata
metodo_atualizacao = 1; %0 = LRI, pag15 Sastry, 1 = padrao igual do rl_lab.m
metodo_normalizacao = 1; %0 = pag15 Sastry, 1 = normalizacao padrao

NIterMax=20000; % no. de itera??es total de aprendizado
NStepMax=20; %no. de passos/itera?oes para come?ar a atualizar probabilidades
prob_conv = 0.98; %probab para considerar convergencia alcan?ada
neta=0.11;     % taxa de aprendizado
ref_s = 2;
if(metodo_atualizacao)
    ref_i = -2;
else
    ref_i = 0;
end

%% Restri??es

% for i = 1:num_obj %objeto generico (quantidade = 1)
%     obj_qtd(i)=1;
% end
% obj_qtd = [1 2 2 2 2 2 2 2];
% obj_qtd = [2 2 2 2 2];
num_obj2 = sum(obj_qtd); %quantidade de material

%restri??o de ordem de obj/carga: linha = qual zona de carga, coluna = qual
%ordem aquela zona pode ter
for i = 1:num_obj2 %ordem generica (qualquer ordem)
    ordem_c(i,1:num_obj2)=1:num_obj2;
end

%definir ordem_c diferentes das genericas
ordem_c(1,:) = [1 zeros(1,num_obj2-1)]; %primeiro obj tem q ser o primeiro
for i = 2:num_obj2
    ordem_c(i,:) = [0 2:num_obj2];
end

%restri??o de ordem de zonas: linha = qual zona de descarga, coluna = qual
%ordem aquela zona pode ter
for i = 1:num_zonas %ordem generica (qualquer ordem)
    for j = 1:num_obj
        ordem(i,j,1:num_obj2)=1:num_obj2;
    end
end

%definir ordens diferentes das genericas
% ordem(1,1,:) = [1 zeros(1,num_obj2-1)]; %primeiro obj tem q ser o primeiro na primeira zona
% for i = 2:num_obj
%     ordem(1,i,:) = [0 2:num_obj2];
% end

%definir demanda das zonas de descarga
for i = 1:num_zonas %demanda generica (demanda max para todas as zonas)
    demanda_zona(i)=num_obj2;
end
% demanda_zona = [2 2 2]; %nao colocar 0, ao inves, apagar zona
% demanda_zona = [2 2];
% demanda_full = 0; %torna-se 1 quando a demanda de todas as zonas ? atendida
obj_sobra = max(num_obj2 - sum(demanda_zona), 0); %numero de objetos que sobrar?o

%objetos aceitos por cada zona e quantidade dos mesmos
for i = 1:num_zonas
    tipo_qtd_zona(i,1:num_obj)=demanda_zona(i);
end
%definir tipo_qtd_zona diferente do generico
% tipo_qtd_zona = [1 0 1 0 0; 2 2 2 2 2; 2 2 2 2 2; 2 2 2 2 2];

%% Defini?ao dos vetores de probabilidades para i-esima a?ao de carga
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
    carga(k) = 1; %numero de objetos que podem ser carregados pelo drone
    for i = 1:num_obj2
        if(num_obj2-i>=carga(k)-1)
            p_carga(i,:,k) = (1/carga(k))*ones(1,carga(k));
        else
            p_carga(i,1:(num_obj2-i+1),k) = (1/(num_obj2-i+1))*ones(1,(num_obj2-i+1));
        end
    end
end

%% Vari?veis

k=1;
dist_min = 1e10;
dist_media = 1e10;
dist = 0;
dist_media = 0;
best = 0;
x = 0; dist_hist = 0; reforco = 0;

z1=[0 0 0 0];
z2=[0 0 0 0];
z3=[0 0 0 0];
z4=[0 0 0 0];

erros_tipo_qtd = [];
erros_ordem = [];
erros_ordem_c = [];
erros = [];

%cond1 = sum(sum(max(prob1,[],2)<prob_conv)>obj_sobra);
%cond2 = sum(sum(max(prob2,[],2)<prob_conv)>obj_sobra);
cond1 = sum(sum(max(prob1,[],2)<prob_conv))>obj_sobra+(num_transp-1)*num_obj2;
cond2 = sum(sum(max(prob2,[],2)<prob_conv))>obj_sobra+(num_transp-1)*num_obj2;

log_count = 0;
A_count = 0;
reforco_zero = 1; %condicional para parar treinamento caso refor?o seja 0 por 20 itera?oes
count_ref_zero = 0;
n_ref_zero = 25;
%(round(dist_media)*1e4/1e4)~=(round(dist)*1e4/1e4)

%% In?cio do treinamento
while (k <= NIterMax && ((cond1 || cond2) && reforco_zero ))
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
        waypoints{n} = [];
        count_wp(n) = 1;
        teta(n) = 0;
%         em_carga(n) = 0;
        finished(n) = 0; %indicador se transp terminou atividades
    end
    dist_fake = zeros(1,n);
    prob_temp1 = prob1;
    prob_temp2 = prob2;
    carga_zona = zeros(1,num_zonas);
    count_descarga = 0;
    count_steps = 0;
    
    e_ordem = 0;
    e_ordem_c = 0;
    e_tipo_qtd = 0;
    obj_qtd_temp = obj_qtd;
    tipo_qtd_zona_temp = tipo_qtd_zona;
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
        
        %selection of n
        n = find(dist_fake==min(dist_fake),1);
        for n = n
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
                    gsuc(n) = 0;
                else   
                    [teta(n), gsuc(n)] = fake_Astar(i1(n),j1(n),i2(n),j2(n),teta(n)); 
                end
                if(debug_mov) disp("transp "+n+": indo de "+pos1(n)+" para "+pos2(n)); end
                dist(n) = dist(n) + gsuc(n);
                pos1(n) = pos2(n);
                [i1(n),j1(n)]=pos_to_ij(pos1(n));
                
                dist_fake(n) = dist_fake(n) + 2*gsuc(n);
                dist_fake = dist_fake - gsuc(n);
                
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
                if(debug) disp("transp "+n+" nao tem atividades");end
                dist_fake(n) = 1e10;
                count2(n) = count2(n)+1;
                if(min(count2)==20)
                    disp("erro: loop infinito")
                    return
                end
            end
        end
        count_steps = count_steps + 1;
        if((e_ordem||e_ordem_c||e_tipo_qtd)>0)
            dist(:)=0;
            count_steps = 0;
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
    if(~(e_ordem || e_ordem_c || e_tipo_qtd))
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
            if(reforco(k)<1e-5)
                count_ref_zero = count_ref_zero + 1;
                if(count_ref_zero == n_ref_zero)
                    reforco_zero = 0;
                end
            else
                count_ref_zero = 0;
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
        erros = cat(2, erros_ordem, erros_ordem_c);
        erros = cat(2, erros, erros_tipo_qtd);
        reforco(k) = 0; %old: -8
%         if(e_ordem)
%             prob1(k2,mov1(k2))=(1+neta*reforco(k))*prob1(k2,mov1(k2));
%             if prob1(k2,mov1(k2)) < 0
%                prob1(k2,mov1(k2))=0;
%             end
%             prob1(k2,:)=prob1(k2,:)/sum(prob1(k2,:)); % normaliza??o
%         end
%         if(e_zona)
%             %diminui prob de objeto ser descarregado na zona incorreta
%             index = find(ismember(obj2(k3-carga_iter+1:k3),tipo_zona(mov2(k3),:))==0);
%             index = k3-carga_iter+index;
%             for i = 1:length(index)
%                 prob2(obj2(index(i)),mov2(k3))=(1+neta*reforco(k))*prob2(obj2(index(i)),mov2(k3));
%                 %prob2(mov1(index(i)),mov2(k3)) = 0.001;
%                 if prob2(obj2(index(i)),mov2(k3)) < 0
%                    prob2(obj2(index(i)),mov2(k3))=0;
%                 end
%                 prob2(obj2(index(i)),:)=prob2(obj2(index(i)),:)/sum(prob2(obj2(index(i)),:)); % normaliza??o
%             end
%             %diminui prob de carregar mais de 1 objeto
% %             if(carga_iter>1)
% %                 i=k3-carga_iter+1;
% %                 p_carga(i,num_carga(i))=(1+neta*reforco(k))*p_carga(i,num_carga(i));
% %                 if p_carga(i,num_carga(i)) < 0
% %                    p_carga(i,num_carga(i)) = 0;
% %                 end
% %                 p_carga(i,:)=p_carga(i,:)/sum(p_carga(i,:)); % normaliza??o
% %             end
%         end
    end
%     prob2
    %prob1(1:ceil(sum(demanda_zona)/num_transp),:,:)
%     if(carga>1)
%         p_carga(1:ceil(sum(demanda_zona)/num_transp),:,:)
%     end
    
    for n=1:num_transp          
        aut1(n) = max(max(prob1(:,:,n),[],2));
        aut2(n) = max(max(prob2(:,:,n),[],2));
        aux = min(max(prob1(find(max(prob1(:,:,n),[],2)>(1.3/num_obj)),:,n),[],2));
        if(isempty(aux))
            aut1min(n) = 1/num_obj;
        else
            aut1min(n) = min(max(prob1(find(max(prob1(:,:,n),[],2)>(1.3/num_obj)),:,n),[],2));
        end
        aux = min(min(prob2(find(max(prob2(:,:,n),[],2)>(1.3/num_obj)),:,n),[],2));
        if(isempty(aux))
            aut2min(n) = 1/num_zonas;
        else
            aut2min(n) = min(max(prob2(find(max(prob2(:,:,n),[],2)>(1.3/num_obj)),:,n),[],2));
        end       
        Tcgmin(n,k) = min(aut1(n),aut2(n));
        Tcgmax(n,k) = max(aut1(n),aut2(n));
        Tcgmed(n,k) = (aut1(n)+aut2(n))/2;
        Tcgmin2(n,k) = min(aut1min(n),aut2min(n));
        Tcgmax2(n,k) = max(aut1min(n),aut2min(n));
        Tcgmed2(n,k) = (aut1min(n)+aut2min(n))/2;
    end
    clear p q;
    %y(k,1)=mov(1);
    %y(k,2)=mov(2);
    %y(k,3)=mov(3);
    oo = sum(sum(max(prob1,[],2)<prob_conv))-(obj_sobra+(num_transp-1)*num_obj2);
    pp = sum(sum(max(prob2,[],2)<prob_conv))-(obj_sobra+(num_transp-1)*num_obj2);
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

%num_carga_hist_last = num_carga_hist(end-5:end,:,1:5)

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
% disp("tempo/iter: "+(elasped_second1/k))
disp("dist: "+dist_hist(end))
disp("dist min: "+best(end))

tempozz(zz+1) = elasped_second1;
n_iterzz(zz+1) = k;
distzz(zz+1) = dist_hist(end);
dist_minzz(zz+1) = best(end);
% plotMap
% prob1(1:ceil(sum(demanda_zona)/num_transp)+2,:,:) %%comentado para 1
% transp
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
%

if(show_plots)
    f1 = figure(1);
    figure(f1)
    a=plot(x(best~=999),best(best~=999),'g');
    hold on
    b=plot(setdiff(x,erros),dist_hist(dist_hist~=0 & dist_hist~=999));
    xlabel("n?mero de itera??es")
    ylabel("dist?ncia total percorrida")
    title("dist?ncia total percorrida em cada itera??o")
    legend([b,a],"dist?ncia total percorrida","dist?ncia m?nima encontrada")

    % f2 = figure(2);
    % figure(f2)
    % b=plot(setdiff(x,erros),count_steps_hist(count_steps_hist~=0));
    % xlabel("n?mero de itera??es")
    % ylabel("n?mero de movimentos")
    % title("n?mero de movimentos em cada itera??o")
    % 
    figure(3)
    plot(x,reforco)
    xlabel("n?mero de itera??es")
    ylabel("valor de refor?o")
    title("valor do refor?o em cada itera??o")
    pause()
    % figure(4)
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

%     for n=1:num_transp
%         figure(4+n)
%         plot(x, Tcgmin(n,:),'b')
%         hold on
%         plot(x, Tcgmed(n,:),'r')
%         plot(x, Tcgmax(n,:),'k')
%         legend("Tcgmin","Tcgmed","Tcgmax")
%         title("Taxa de convergencia geral (Tcg), transp "+n) %atualizacao="+metodo_atualizacao+", normalizacao="+metodo_normalizacao)
%         grid
%     end
    
    for n=1:num_transp
        figure(4+n)
        plot(x, Tcgmax(n,:),'b')
        hold on
        plot(x, Tcgmax2(n,:),'k')
        legend("Tcgmin","Tcgmax")
        title("Taxa de convergencia geral (Tcg), transp "+n) %atualizacao="+metodo_atualizacao+", normalizacao="+metodo_normalizacao)
        grid
    end
end
close all
% [dist,waypoints2]=rota_multi(prob1,prob2,p_carga, obj_qtd,demanda_zona);
end

sprintf('%.2f ', tempozz)
sprintf('%.2f ', n_iterzz)
sprintf('%.2f ', distzz)
sprintf('%.2f ', dist_minzz)

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
    global num_carga e_ordem_c erros_ordem_c ordem_c_temp
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
        obj_qtd_temp(mov1(n,k2(n)))=obj_qtd_temp(mov1(n,k2(n)))-1;
        if(debug) disp("obj_qtd_temp: "+obj_qtd_temp); end
        obj2(n,k2(n)) = sum(obj_qtd(1:mov1(n,k2(n))))-obj_qtd_temp(mov1(n,k2(n)));
        
        %erro de ordem_c
%         if(~ismember(k2(n),ordem_c_temp(obj2(n,k2(n)),:)))
%             e_ordem_c = n; 
%             erros_ordem_c(end+1) = k; %salva itera?ao que deu erro
%         end
        
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
        k2(n) = k2(n) + 1;
        for m=1:num_transp
            if(k2(m)<=num_obj2)
                if(sum(prob_temp1(k2(m),:,m))~=0)
                    prob_temp1(k2(m),:,m)=prob_temp1(k2(m),:,m)/sum(prob_temp1(k2(m),:,m)); % normaliza??o
                end
                prob_sum1(k2(m),1,m)=prob_temp1(k2(m),1,m);
                for j=2:num_obj
                    prob_sum1(k2(m),j,m)=prob_sum1(k2(m),j-1,m)+prob_temp1(k2(m),j,m);
                end
            end
        end
    end
end

function pos = decidir_descarga(n)
    global k3 prob_sum2 prob_temp2 num_obj2 num_transp debug
    global num_zonas mov2 carga_zona demanda_zona no_final obj2 %carga_iter
    global ordem mov1 e_ordem erros_ordem num_carga k2 dist k count_steps %variaveis necessarias para erro_ordem
    global e_tipo_qtd erros_tipo_qtd tipo_qtd_zona_temp
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
%         if(k2(n)<num_obj2)
%             mov1(n,k2(n)+1:num_obj2) = 0;
%             mov2(n,k3(n)+1:num_obj2) = 0;
%             num_carga(n,k2(n)+1:num_obj2) = 0;
%             obj2(n,k2(n)+1:num_obj2) = 0;
%         end
    %erro de tipo_qtd_zona
    %zona de descarga escolhida: mov2(n,k3(n))
    %zona de carga a ser descarregada: mov1(n,k3(n))
    %quantos objetos daquele tipo a zona aceita: tipo_qtd_zona_temp(mov2(n,k3(n)),mov1(n,k3(n)))
    elseif(~tipo_qtd_zona_temp(mov2(n,k3(n)),mov1(n,k3(n))))
        if(debug) disp("erro tipo_qtd");end
        e_tipo_qtd = n; 
        erros_tipo_qtd(end+1) = k; %salva itera?ao que deu erro
    else
        carga_zona(mov2(n,k3(n))) = carga_zona(mov2(n,k3(n))) + 1;
        tipo_qtd_zona_temp(mov2(n,k3(n)),mov1(n,k3(n))) = tipo_qtd_zona_temp(mov2(n,k3(n)),mov1(n,k3(n))) - 1;
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

function [tetap, gsuc] = teta_e_custo(teta0, wayps)
    global num_col
    xh = 1; xv = 1; xd = sqrt(2); xteta = 1;
    i_inic = wayps(1,1);
    i_final = wayps(2,1);
	j_inic = wayps(1,2);
    j_final = wayps(2,2);
    if j_final==0
        j_final = num_col; i_final = i_final-1;
    end

    if (i_inic==i_final)
        if (j_final>=j_inic) tetap=0;
        else tetap=180;
        end
    elseif (j_inic==j_final)
        if (i_final<=i_inic) tetap=90;
        else tetap=-90;
        end
    else
        tetap=atan2(-(i_final-i_inic)*xv,(j_final-j_inic)*xh)*180.0/pi;
    end
    dteta=abs(teta0-tetap);
    while (dteta>180.0)
        dteta=abs(dteta-360.0);
    end
    % se n? sucessor est? na mesma linha de N */
    if ((i_inic==i_final) && (j_inic~=j_final))
        gsuc=xh;
    end
    % se n? sucessor est? na mesma coluna de N */
    if ((j_inic==j_final) && (i_inic~=i_final))
        gsuc=xv;
    end
    % se n? sucessor n?o na mesma linha nem na mesma coluna de N */
    if ((i_inic~=i_final) && (j_inic~=j_final))
        gsuc=xd;
    end
    gsuc=gsuc+xteta*dteta/90.0;

end

function [teta_out, gsuc] = fake_Astar(i1, j1, i2, j2, teta_in)
    gsuc = pdist([i1 j1;i2 j2]);
    teta_out = atan2(-(i1-i2),(j1-j2))*180.0/pi;
    dteta = abs(teta_out - teta_in);
    gsuc = gsuc + dteta/90;
end