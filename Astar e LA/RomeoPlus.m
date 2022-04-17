%{
        Programa RomeoPlus.m, adaptado de ROMEO.cpp
        O programa usa o algoritmo A* para determinar
        rota entre nó inicial e final dentro de uma grade

        Responsável por ROMEO.cpp: Cairo L. Nascimento Jr.
        Autor de RomeoPlus.m: Walber Lima P. Junior
        27/08/19
%}

close all; clear all;
clc;
% axis equal;
% axis ij
% grid

%% VARIAVEIS
global num_lin num_col i_final j_final CLOSED nclosed map teta0 fim_da_busca nclosed_aux
global GCLOSED nopen OPEN GOPEN gsuc h i_inic j_inic xh xv xd xteta not_yet custo_rota CLOSED_aux
global debug_mode
debug_mode = 0;

no_inic = 0;
xh = 1; %custo do movimento horizontal
xv = 1; %custo do movimento vertical
%r = xh/xv;
xteta = 1; %custo do movimento angular de 90 graus
xd = sqrt(xh^2+xv^2); %custo do movimento diagonal
teta0 = 0; %angulo inicial
tc = .4; %tamanho/raio do circulo plotado
not_yet = 0;
%% LER E PLOTAR LABIRINTO
% file = fopen('activemap.txt','r');
% size = fscanf(file,'%d',[1 2]);

tic;
load('map.mat');
map(map==2)=0;
map(map==3)=0;
% map = zeros(780,780);
figure(1)
axis([0 800 0 800])
hold on
grid on
% map(50,1:40) = 1;
% map(50,50:90) = 1;
map(390,468) = 2;
map(78,702) = 3;
plot(390,468,'b*')
plot(78,702,'b*')
num_lin = size(map,1);
num_col = size(map,2);


% map = fscanf(file,'%d',[num_col num_lin]);
% map = map';


%ax = gca;
%ax.DataAspectRatio = [1 xh/xv 1];

%ax.GridColor = [0 0 0];
%ax.GridLineStyle = '--';
%ax.GridAlpha = 0.2;
%ax.Layer = 'top';
% xticks(1:num_col)
% yticks(1:num_lin)
%xlim([0.25 num_col+0.75])
%ylim([0.25 num_lin+0.75])
% axis([0 num_col+1 0 num_lin+1])

%rectangle('Position',[j-tc/2,i-tc/2,tc,tc],'Curvature',[1 1])

for i = 1:num_lin
    for j = 1:num_col
        if map(i,j) == 0  %Espaço vazio
%             rectangle('Position',[j-tc/2,i-tc/2,tc,tc],'Curvature',[1 1])
        end
        if map(i,j) == 1  %Espaço com obstaculo
%             rectangle('Position',[j-tc/2,i-tc/2,tc,tc],'Curvature',[1 1],'FaceColor',[0.4 0.4 0.4])
        end
        if map(i,j) == 2  %Nó inicial
            no_inic=1; i_inic = i; j_inic = j;
%             rectangle('Position',[j-tc/2,i-tc/2,tc,tc],'Curvature',[1 1],'FaceColor',[0 0 1])
        end
        if map(i,j) == 3  %Nó final
            no_final=1; i_final = i; j_final = j;
%             rectangle('Position',[j-tc/2,i-tc/2,tc,tc],'Curvature',[1 1],'FaceColor',[0 1 0])
        end
    end
end

% fclose(file);

%% MATRIZ H (custo de cada nó para nó final)
h = zeros(num_col,num_lin);
%h = zeros(1,num_lin*num_col);
for i = 1:num_lin
    for j = 1:num_col
        dist_hor = abs(j_final-j)*xh;
        dist_vert = abs(i_final-i)*xv;
        %h(num_col*(i-1)+j) = sqrt(dist_hor^2 + dist_vert^2);
%         if(dist_hor ~= 0 && dist_vert ~=0)
%             aux = (dist_hor~=dist_vert)*xteta/2;
%         else
%             aux = 0;
%         end
        h(num_col*(i-1)+j) = (dist_hor + dist_vert) + (xd - 2) * min(dist_hor, dist_vert);% + aux;
        %h(num_col*(i-1)+j) = h(num_col*(i-1)+j)*2;
    end
end
%h=h';
%% PLANEJAR TRAJETÓRIA (A*)

OPEN(1,1) = num_col*(i_inic-1)+j_inic; %nó inicial vai para OPEN
OPEN(1,2) = -1; %antecessor do nó inicial = -1
GOPEN(1) = 0; %custo inicial = 0
nopen=1;
nclosed=0;
fim_da_busca=0;
npt=1;  % numero de pontos na trajetoria

while 1
    %Toma o primeiro elemento de OPEN, chamando-o de N
    N=OPEN(1,1);
    gn=GOPEN(1); % Toma o seu custo gn do nó N
    %Retira esse elemento de OPEN e o coloca em CLOSED */
    %disp("nopen: "+nopen+"  nclosed: "+nclosed)
    %disp("nó "+N+" indo de open para closed")
    CLOSED(nclosed+1,1)=N;
    CLOSED(nclosed+1,2)=OPEN(1,2);
    GCLOSED(nclosed+1)=GOPEN(1);
    nclosed = nclosed + 1;
    nopen = nopen - 1;
    %disp("Nó "+N)
    %if nopen>0
    %    for i=1:nopen
    %        disp("OPEN("+i+",1)= "+OPEN(i,1)+" OPEN("+i+",2)= "+OPEN(i,2))
    %    end
    %end
    %if nclosed>0
    %    for i=1:nclosed
    %        disp("CLOSED("+i+",1)= "+CLOSED(i,1)+" CLOSED("+i+",2)= "+CLOSED(i,2))
    %    end
    %end
    %fprintf("\n\n")
    % Desloca os elementos da matriz OPEN e do vetor GOPEN visto que
    % o primeiro elemento de OPEN foi transferido para CLOSED */
    if nopen>0
        %disp("organizando OPEN...")
        for i = 1:nopen
            OPEN(i,1)=OPEN(i+1,1);
            OPEN(i,2)=OPEN(i+1,2);
            GOPEN(i)=GOPEN(i+1);
            %disp("OPEN("+i+",1)= "+OPEN(i,1))
            %disp("OPEN("+i+",2)= "+OPEN(i,2))
        end
    end
    %Se N ainda não é o nó final, N é expandido
    if (N~=((i_final-1)*num_col+j_final))
        %disp("nopen: "+nopen+"  nclosed: "+nclosed)
        %fprintf("\n")
        i1=fix(N/num_col)+1;
        j1=mod(N,num_col);
        if j1==0
            j1 = num_col; i1 = i1-1;
        end
        %disp("expandindo nó ("+i1+","+j1+")...")
        plot(i1,j1,'ko','MarkerSize',1.5)
        drawnow
        %disp(" ")
        expand(N,gn);
        %pause(5)
        reord_open(); % Reordena OPEN com base no custo f'=g+h
    end
    %fprintf("\n\n\n")
    if (fim_da_busca==1 || nopen==0)
        if(nopen==0 && not_yet==0)
            disp("nao foi encontrada nenhuma rota")
            break
        else
            if(not_yet == 1 && nopen~=0 && (custo_rota(1)<custo_rota(2)))
                disp("encontrada duas rotas, primeira foi melhor")
                CLOSED = CLOSED_aux; %primeira rota foi melhor, voltar a ela
                nclosed = nclosed_aux;
                gsuc = custo_rota(1);
            else
                if(not_yet==1 && nopen~=0)
                    disp("encontrada duas rotas, segunda foi melhor")
                    CLOSED(nclosed+1,2)=N;
                    CLOSED(nclosed+1,1)=(i_final-1)*num_col+j_final;
                    nclosed = nclosed + 1;
                    gsuc = custo_rota(2);
                else
                    if(not_yet==1)
                        disp("nao foi encontrada segunda rota")
                        CLOSED = CLOSED_aux; %manter primeira rota
                        nclosed = nclosed_aux;
                        gsuc = custo_rota(1);
                    else %not_yet 0 e nopen >0
                        disp("só foi necessária uma rota")
                        CLOSED(nclosed+1,2)=N; %só precisou de uma rota
                        CLOSED(nclosed+1,1)=(i_final-1)*num_col+j_final;
                        nclosed = nclosed + 1;
                        gsuc = custo_rota(1);
                    end
                end
            end
%             desenhar_caminho()
            break
        end
    end
end
disp("gsuc: "+gsuc)
toc
nclosed
%% Função expand

function expand(N, gn)
global num_lin num_col i_final j_final CLOSED nclosed map teta0 fim_da_busca
global GCLOSED nopen OPEN GOPEN gsuc h xh xv xd xteta not_yet custo_rota CLOSED_aux nclosed_aux
global debug_mode
in=fix(N/num_col)+1;  % in,jn = posição do nó N na grade */
jn=mod(N,num_col);
if jn==0
    jn = num_col; in = in-1;
end

Np=CLOSED(nclosed,2);  % recupera o pai do nó N = Np*/
if (in>=1) && (jn>=1) && (in<=num_lin) && (jn<=num_col)
    if map(in,jn)==2
        ip=-1; jp=-1; % ip,jp = posição do nó Np na grade */
    else
        ip=fix(Np/num_col)+1;
        jp=mod(Np,num_col);
        if jp==0
            jp = num_col; ip = ip-1;
        end
    end
    
    if (map(in,jn)==2)
        tetac=teta0; % se for nó de partida */
    else
        tetac=atan2(-(in-ip)*xv,(jn-jp)*xh)*180.0/pi;
    end
end

for i=in-1:in+1
    for j=jn-1:jn+1
        if(debug_mode) disp("expanding ("+i+","+j+")..."); end
        suc=num_col*(i-1)+j;
        %regra de validade do novo n¢ */
        if ( (i>=1) && (j>=1) && (i<=num_lin) && (j<=num_col) && (map(i,j)~=1) && (suc~=N) && (CLOSED(nclosed,2)~=suc) && (fim_da_busca==0))
            %disp("suc= "+suc+", CLOSED(nclosed,2)= "+CLOSED(nclosed,2))
            %Calcular o ƒngulo de partida tetap considerando o
            %movimento de (in,jn) para (i,j) e que estamos em
            %(in,jn)
            if (i==in)
                if (j>=jn) tetap=0;
                else tetap=180;
                end
            elseif (j==jn)
                if (i<=in) tetap=90;
                else tetap=-90;
                end
            else
                tetap=atan2(-(i-in)*xv,(j-jn)*xh)*180.0/pi;
            end
            
            dteta=abs(tetac-tetap);
            while (dteta>180.0)
                dteta=abs(dteta-360.0);
            end
            % se n¢ sucessor est  na mesma linha de N */
            if ((i==in) && (j~=jn))
                gsuc=gn+xh;
            end
            % se n¢ sucessor est  na mesma coluna de N */
            if ((j==jn) && (i~=in))
                gsuc=gn+xv;
            end
            % se n¢ sucessor nÆo na mesma linha nem na mesma coluna de N */
            if ((i~=in) && (j~=jn))
                gsuc=gn+xd;
            end
            gsuc=gsuc+xteta*dteta/90.0;
            
            %se o n¢ sucessor for o n¢ final entÆo indicar com o
            %flag fim_da_busca=1 e armazenar o valor de gn como sendo
            %o custo total da rota
            if ((i==i_final) && (j==j_final))
                if(dteta>0 && not_yet==0)
                    disp("rota terminou em rotaçao, encontrando outra rota")
                    fim_da_busca=0; %ainda será encontrada mais uma rota
                    not_yet = 1;
                    pause(3);
                    CLOSED_aux = CLOSED;
                    nclosed_aux = nclosed + 1;
                    CLOSED_aux(nclosed+1,2)=N;
                    CLOSED_aux(nclosed+1,1)=(i_final-1)*num_col+j_final;
                else
                    disp("encontrada rota")
                    pause(3);
                    fim_da_busca=1; %fim
                end
                custo_rota(not_yet+fim_da_busca)=gsuc;      % custo total da rota      */
            end
            
            novo=1;
            
            %disp("checando se sucessor ja está em OPEN...")
            for a=1:nopen
                %disp("suc: "+suc+", OPEN(a,1): "+OPEN(a,1))
                if suc==OPEN(a,1)
                    if(debug_mode) disp("sucessor ("+i+","+j+") já existe em OPEN"); end
                    novo=0;
                    if (gsuc<GOPEN(a))
                        if(debug_mode) disp("mantendo o SUC"); end
                        GOPEN(a)=gsuc;
                        OPEN(a,2)=N;
                        %desenhar_circ(cor[2]); /* cor verde */
                    else
                        if(debug_mode) disp("mantendo o OPEN"); end
                        gsuc=GOPEN(a);
                        %desenhar_circ(cor[1]); /* cor vermelha */
                    end
                end
            end
            
            for a=1:nclosed
                if (suc==CLOSED(a,1))
                    if(debug_mode) disp("sucessor ("+i+","+j+") já existe em CLOSED"); end
                    novo=0;
                    if (gsuc<GCLOSED(a))
                        if(debug_mode) disp("mantendo o SUC"); end
                        GCLOSED(a)=gsuc;
                        CLOSED(a,2)=N;
                        %desenhar_circ(cor[2]); /* cor verde */
                    else
                        if(debug_mode) disp("mantendo o CLOSED"); end
                        gsuc=GCLOSED(a);
                        %desenhar_circ(cor[1]); /* cor vermelha */
                    end
                end
            end % end do for e do if */
            
            % Se suc nao se encontra em OPEN nem em CLOSED, ele eh adicionado
            % a OPEN */
            if (novo==1)
                if(debug_mode) disp("sucessor ("+i+","+j+") é novo"); end
                OPEN(nopen+1,1)=suc;
                GOPEN(nopen+1)=gsuc;
                OPEN(nopen+1,2)=N;
                nopen = nopen + 1;
                %desenhar_circ(cor[0]); /* cor branca */
            end % end do if */
        end
    end
end
end


%% FUNÇÃO REORD_OPEN

function reord_open()
global OPEN GOPEN nopen h
TEMP1(1,1)=OPEN(1,1);
TEMP1(1,2)=OPEN(1,2);
TEMP2(1)=GOPEN(1);
%disp("nopen: "+nopen)
for a=2:nopen
    N=OPEN(a,1);
    fn = GOPEN(a)+h(N);
    %fn = round((GOPEN(a)+h(N))*1e7)/1e7;
    flag=0;
    b=1;
    while b<=(a-1)
        %if(N==85 || N==28)
        %    disp("F do nó "+N+": "+fn)
        %    disp("G do nó "+N+": "+GOPEN(a))
        %    disp("H do nó "+N+": "+h(N))
        %    disp("F do nó "+TEMP1(b,1)+": "+(TEMP2(b)+h(TEMP1(b,1))))
        %    disp("G do nó "+TEMP1(b,1)+": "+TEMP2(b))
        %    disp("H do nó "+TEMP1(b,1)+": "+h(TEMP1(b,1)))
        %    fn==round(TEMP2(b)+h(TEMP1(b,1)),10)
        %end
        %disp("fn: "+fn+", GOPEN("+b+"): "+GOPEN(b)+" , h("+b+"): "+h(OPEN(b)))
        fn2 = TEMP2(b)+h(TEMP1(b,1));
        %fn2 = round((TEMP2(b)+h(TEMP1(b,1)))*1e7)/1e7;    
        if ((fn<fn2 || ((fn==fn2)&&(GOPEN(a)<TEMP2(b)))))
            for c=(a):-1:(b+1)
                %disp("a: "+a+"  b: "+b+"  c: "+c)
                TEMP1(c,1)=TEMP1(c-1,1);
                TEMP1(c,2)=TEMP1(c-1,2);
                TEMP2(c)=TEMP2(c-1);
            end
            TEMP1(b,1)=N;
            TEMP1(b,2)=OPEN(a,2);
            TEMP2(b)=GOPEN(a);
            flag=1;
            b=a;
        end
        b=b+1;
    end
    if flag==0
        TEMP1(a,1)=N;
        TEMP1(a,2)=OPEN(a,2);
        TEMP2(a)=GOPEN(a);
    end
end
for a=1:nopen
    OPEN(a,1)=TEMP1(a,1);
    OPEN(a,2)=TEMP1(a,2);
    GOPEN(a)=TEMP2(a);
end
end

function desenhar_caminho()
global nclosed CLOSED num_col OPEN i_final j_final i_inic j_inic xh xv xd xteta
N = (i_final-1)*num_col + j_final;
Nf = (i_inic-1)*num_col + j_inic;
i = 1;
while(i<=nclosed)
    if(CLOSED(i,1)==N)
        in=fix(CLOSED(i,1)/num_col)+1;
        jn=mod(CLOSED(i,1),num_col);
        if jn==0
            jn = num_col; in = in-1;
        end
        in2=fix(CLOSED(i,2)/num_col)+1;
        jn2=mod(CLOSED(i,2),num_col);
        if jn2==0
            jn2 = num_col; in2 = in2-1;
        end
        x=[in in2];
        y=[jn jn2];
    	a = line(y,x);
        a.LineWidth = 2;
        N = CLOSED(i,2);
        i = 0;
        if(N==Nf)
            break
        end
    end
    i = i + 1;
end

end
