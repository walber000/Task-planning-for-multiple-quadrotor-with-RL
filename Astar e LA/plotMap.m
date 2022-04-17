
mapa = figure();
%f2=figure;
%mapa = subplot(2,3,[1 2 4 5]);
%mapa.Position = [0.1 0.11 0.4942 0.815];

%mapa.WindowState = 'maximized';

axis equal;
axis ij
grid

%% VARIAVEIS
tc = .4; %tamanho/raio do circulo plotado

%% LER E PLOTAR LABIRINTO
% file = fopen('activemap.txt','r');
file = fopen('activemap_multi.txt','r');
size = fscanf(file,'%d',[1 2]);
num_lin = size(1,1);
num_col = size(1,2);
map = fscanf(file,'%d',[num_col num_lin]);
map = map';

%figure('units','normalized','outerposition',[0 0 1 1])

%frame_h = get(handle(gcf),'JavaFrame');
%set(frame_h,'Maximized',1);

xticks(1:num_col)
yticks(1:num_lin)
axis([0 num_col+1 0 num_lin+1])

% mapa.Position = [0.1300 0.5838 0.7750 0.3412];

k=1; obj=1; num_transp = 0;
dx = 0.12; dy = 0.18;
font_size = 7;
texto=sprintf('%d',k);
for i = 1:num_lin
    for j = 1:num_col
        if map(i,j) == 0  %Espaço vazio
            rectangle('Position',[j-tc/2,i-tc/2,tc,tc],'Curvature',[1 1])
        end
        if map(i,j) == 1  %Espaço com obstaculo
            rectangle('Position',[j-tc/2,i-tc/2,tc,tc],'Curvature',[1 1],'FaceColor',[0.4 0.4 0.4])
        end
        if map(i,j) == 2  %Nó inicial
            rectangle('Position',[j-tc/2,i-tc/2,tc,tc],'Curvature',[1 1],'FaceColor',[0 0 1])
            num_transp = num_transp + 1;
            texto=sprintf('%d',num_transp);
            t=text(j-tc/2+dx,i-tc/2+dy,texto,'Color','white','FontSize',font_size);
        end
        if map(i,j) == 3  %Objetos
            rectangle('Position',[j-tc/2,i-tc/2,tc,tc],'Curvature',[1 1],'FaceColor',[0 1 0])
            texto=sprintf('%d',obj);
            t=text(j-tc/2+dx,i-tc/2+dy,texto,'Color','black','FontSize',font_size);
            obj=obj+1;
        end
        if map(i,j) == 4  %Ponto de entrega
            rectangle('Position',[j-tc/2,i-tc/2,tc,tc],'Curvature',[1 1],'FaceColor',[1 1 0])
            %text(j-tc/2+0.1,i-tc/2-0.3,texto,'FontSize',font_size);
            texto=sprintf('%d',k);
            t=text(j-tc/2+dx,i-tc/2+dy,texto,'Color','red','FontSize',font_size);
            k=k+1; 
            %delete(t)
            %t=text(j-tc/2+dx,i-tc/2+0.18,texto,'Color','red','FontSize',font_size);
        end
    end
end
%set(gcf, 'Position', get(0, 'Screensize'));
fclose(file);