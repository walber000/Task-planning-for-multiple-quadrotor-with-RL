%{
        Programa RomeoPlus.m, adaptado de ROMEO.cpp
        O programa usa o algoritmo A* para determinar
        rota entre nó inicial e final dentro de uma grade

        Responsável por ROMEO.cpp: Cairo L. Nascimento Jr.
        Autor de RomeoPlus.m: Walber Lima P. Junior
        27/08/19
%}

function [gsuc, count, linha, waypoints] = Astar(start, goal, desenhar, cor)
%% VARIAVEIS
i_inic = start(1);
j_inic = start(2);
i_final = goal(1);
j_final = goal(2);
xh = 1; %custo do movimento horizontal
xv = 1; %custo do movimento vertical
%r = xh/xv;
xteta = 1; %custo do movimento angular de 90 graus
xd = sqrt(xh^2+xv^2); %custo do movimento diagonal
teta0 = 0; %angulo inicial
%tc = .4; %tamanho/raio do circulo plotado
gsuc = 0;
linha = 0;
not_yet = 0; custo_rota = 0; CLOSED_aux = 0; nclosed_aux = 0;
waypoints = [];


%% LER E PLOTAR LABIRINTO
file = fopen('activemap.txt','r');
size = fscanf(file,'%d',[1 2]);

% map = zeros(250,250);
% num_lin = size(map,1);
% num_col = size(map,2);

num_lin = size(1,1);
num_col = size(1,2);
map = fscanf(file,'%d',[num_col num_lin]);
map = map';

%rectangle('Position',[j-tc/2,i-tc/2,tc,tc],'Curvature',[1 1])
for i = 1:num_lin
    for j = 1:num_col
        % if map(i,j) == 0 -> espaço vazio
        % if map(i,j) == 1 -> obstaculo
        if map(i,j) == 2  %Nó inicial do mapa global vira espaço vazio
            map(i,j) = 0;
        end
        if map(i,j) == 3  %Nó final do mapa global vira espaço vazio
            map(i,j) = 0;
        end
        if map(i,j) == 4  %Ponto de entrega do mapa global vira espaço vazio
            map(i,j) = 0;
        end
    end
end

map(i_inic, j_inic) = 2;
map(i_final, j_final) = 3;

% fclose(file);

%% MATRIZ H (custo de cada nó para nó final)
h = zeros(num_col,num_lin);
for i = 1:num_lin
    for j = 1:num_col
        dist_hor = abs(j_final-j)*xh;
        dist_vert = abs(i_final-i)*xv;
        %h(num_col*(i-1)+j) = sqrt(dist_hor^2 + dist_vert^2);
        h(num_col*(i-1)+j) = (dist_hor + dist_vert) + (xd - 2) * min(dist_hor, dist_vert);
    end
end
%% PLANEJAR TRAJETÓRIA (A*)

OPEN(1,1) = num_col*(i_inic-1)+j_inic; %nó inicial vai para OPEN
OPEN(1,2) = -1; %antecessor do nó inicial = -1
GOPEN(1) = 0; %custo inicial = 0
nopen=1;
nclosed=0;
fim_da_busca=0;
%npt=1;  % numero de pontos na trajetoria

while 1
    %Toma o primeiro elemento de OPEN, chamando-o de N
    N=OPEN(1,1);
    gn=GOPEN(1); % Toma o seu custo gn do nó N

    CLOSED(nclosed+1,1)=N;
    CLOSED(nclosed+1,2)=OPEN(1,2);
    GCLOSED(nclosed+1)=GOPEN(1);
    nclosed = nclosed + 1;
    nopen = nopen - 1;

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
        %disp("expandindo nó "+N)
%         if(nclosed>600)
%             disp("wait")
%             pause(3)
%         end
        N2=N;
        expand(N,gn);
        %pause(5)
        reord_open(); % Reordena OPEN com base no custo f'=g+h
    end
    %fprintf("\n\n\n")
    if (fim_da_busca==1 || nopen==0)
        if(not_yet == 0 && nopen ~= 0) %achou apenas uma rota
            CLOSED(nclosed+1,2)=N2; %só precisou de uma rota
            CLOSED(nclosed+1,1)=(i_final-1)*num_col+j_final;
            nclosed = nclosed + 1;
            gsuc = custo_rota(1);
        else
            if(not_yet == 1) %duas rotas
                if(nopen~=0) %encontradas duas rotas
                    if(custo_rota(1)<custo_rota(2))
                        CLOSED = CLOSED_aux; %primeira rota foi melhor, voltar a ela
                        nclosed = nclosed_aux;
                        gsuc = custo_rota(1);
                    else
                        CLOSED(nclosed+1,2)=N2; %segunda rota foi melhor
                        CLOSED(nclosed+1,1)=(i_final-1)*num_col+j_final;
                        nclosed = nclosed + 1;
                        gsuc = custo_rota(2);
                    end
                else
                    CLOSED = CLOSED_aux; %segunda rota deu ruim, manter primeira
                    nclosed = nclosed_aux;
                    gsuc = custo_rota(1);
                end
            else
                break %not_yet = 0 e nopen = 0: nenhuma rota encontrada
            end
        end
        if(desenhar == 1)
            [linha, waypoints] = desenhar_caminho();
        else
            count = 0;
        end
        break
    end
end


%% Função expand

function expand(N, gn)

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
        %disp("expanding ("+i+","+j+")...")
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
                   fim_da_busca=0; %ainda será encontrada mais uma rota
                   not_yet = 1;
                   CLOSED_aux = CLOSED;
                   nclosed_aux = nclosed + 1;
                   CLOSED_aux(nclosed+1,2)=N;
                   CLOSED_aux(nclosed+1,1)=(i_final-1)*num_col+j_final;
                else
                    fim_da_busca=1; %fim
                end
                custo_rota(not_yet+fim_da_busca)=gsuc;      % custo total da rota      */
            end
            
            novo=1;
            
            %disp("checando se sucessor ja está em OPEN...")
            for a=1:nopen
                %disp("suc: "+suc+", OPEN(a,1): "+OPEN(a,1))
                if suc==OPEN(a,1)
                    %disp("sucessor ("+i+","+j+") já existe em OPEN")
                    novo=0;
                    if (gsuc<GOPEN(a))
                        %disp("mantendo o SUC")
                        GOPEN(a)=gsuc;
                        OPEN(a,2)=N;
                        %desenhar_circ(cor[2]); /* cor verde */
                    else
                        %disp("mantendo o OPEN")
                        gsuc=GOPEN(a);
                        %desenhar_circ(cor[1]); /* cor vermelha */
                    end
                end
            end
            
            for a=1:nclosed
                if (suc==CLOSED(a,1))
                    %disp("sucessor ("+i+","+j+") já existe em CLOSED")
                    novo=0;
                    if (gsuc<GCLOSED(a))
                        %disp("mantendo o SUC")
                        GCLOSED(a)=gsuc;
                        CLOSED(a,2)=N;
                        %desenhar_circ(cor[2]); /* cor verde */
                    else
                        %disp("mantendo o CLOSED")
                        gsuc=GCLOSED(a);
                        %desenhar_circ(cor[1]); /* cor vermelha */
                    end
                end
            end % end do for e do if */
            
            % Se suc nao se encontra em OPEN nem em CLOSED, ele eh adicionado
            % a OPEN */
            if (novo==1)
                %disp("sucessor ("+i+","+j+") é novo")
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
TEMP1(1,1)=OPEN(1,1);
TEMP1(1,2)=OPEN(1,2);
TEMP2(1)=GOPEN(1);
%disp("nopen: "+nopen)
for a=2:nopen
    N=OPEN(a,1);
    fn=GOPEN(a)+h(N);
    %fn=round((GOPEN(a)+h(N))*1e6)/1e6;
    flag=0;
    b=1;
    while b<=(a-1)
        %disp("fn: "+fn+", GOPEN("+b+"): "+GOPEN(b)+" , h("+b+"): "+h(OPEN(b)))
        fn2 = TEMP2(b)+h(TEMP1(b,1));
        %fn2 = round((TEMP2(b)+h(TEMP1(b,1)))*1e6)/1e6;
        if (fn<fn2 || ((fn==fn2)&&(GOPEN(a)<TEMP2(b))))
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

function [linha, waypoints] = desenhar_caminho()
hold on
N = (i_final-1)*num_col + j_final;
Nf = (i_inic-1)*num_col + j_inic;
i = 1;
count = 0;
waypoints(1,:) = [i_final j_final];
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
        waypoints(count+2,:) = [in2 jn2]; 
    	linha(count+1) = line(y,x);
        linha(count+1).LineWidth = 2;
        if(cor==2)
            linha(count+1).Color = [1 0 0];
        end
        count = count + 1;
        N = CLOSED(i,2);
        i = 0;
        if(N==Nf)
            break
        end
    end
    i = i + 1;
end
waypoints = flipud(waypoints);
end
nopen
nclosed
end