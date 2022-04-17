%close all
% clear all

bars = 0;

if(bars)
    n=3;
    
    time_carga = [139.67, 143.5, 249.51];
    time_descarga = [277.47, 111.87, 133.36];
    time_finish = [24.51, 61.16, 55.23];
    time_idle_total = [0, 0, 16.44];
    
    for i=1:n
        y(i,:) = [time_carga(i), time_descarga(i), time_finish(i), time_idle_total(i)];
    end
    figure()
    bar(y,0.65,'stacked')
    legend('em carga','em descarga','em retorno','parado')
    xlabel('transportador')
    ylabel('tempo de execução (s)')
    title('tempo de execução da tarefa para cada transportador') 
    xlim([0.25 5])
    grid
   return 
end

n=10;
results = 1;

if(results==1)
    mapa_iter{1} = [219,262,236,181,239,161,364,262,256];
    mapa_iter{2} = [824	650	595	406	700	352	506	438	614 422];
    mapa_iter{3} = [525	466	940	522	491	503	617	1107	1045	1255];
    mapa_iter{4} = [1372	773	900	861	881	978	1036	968	854	968];
    mapa_iter{5} = [1813	917	1150	1849 1153	1073	953];
    mapa_iter{6} = [1450	1264	1614	2031	1442	1765	1205	1577	1325	1248];
    mapa_iter{7} = [1628,1859,2536,1929,2467,3080,1362,2256,3007,3062];
    mapa_iter{8} = [1970	2490	4707	2384	2037	1784	1868	1769	2624	1932];
    mapa_iter{9} = [2623	2694	3257	2822	2667	2403	2880	3576	2651	2461];
    mapa_iter{10} = [3186	3749	2416	3879	2904	3706	3260	4476 4545];
    
    mapa_time{1} = [5,5.6,5,4.4,4.8,3.8,7,5,5.3];
    mapa_time{2} = [37.4	34.3	37.9	32.2	35.6	29.6	32.4	31	33.8 32];
    mapa_time{3} = [57.9	55.2	68.8	57.8	57.8	60.6	58.4	77.6	85.6	81.2];
    mapa_time{4} = [171.8	151.3	157	154.4	161.9	172.4	166.7	161.6	159.9	157.3];
    mapa_time{5} = [898.02	826.27	839.52	870.32	886.34	831.28	849.3];
    mapa_time{6} = [1147.1	1128.5	1141.3	1188.1	1145.4	1192.2	1163.9	1183.3	1161.9	1146.8];
    mapa_time{7} = [4009.2,3969.6,4004.8,4117.3,4032.8,4105,3796.7,4052.1,4069.7,4075.1];
    mapa_time{8} = [6733.72	6787.4	7011.17	6640.21	6670.85	6873.88	6707.41	6858.22	6807.31	6876.03];
    mapa_time{9} = [12013.45	12270.92	12015.93	12502.8	11505.7	11629.84	11732.5	11884.65	11455.83	11681.41];
    mapa_time{10} = [19730.92	20213.35	19879.81	22591.62	21214.64	20166.3	19436.75	20543.06 20965.7];
    
    mapa_dist{1} = [232.15	200.85	218.57	200.85	200.85 200.85	235.38	236.24	210.85	200.85];
    mapa_dist{2} = [694.09	749.74	686.73	677.6	710.22	694.09	692.59	688.23	718.61 677.1];
    mapa_dist{3} = [1554.74	1606.64	1565.89	1636.05	1571.78	1608.13	1608.06	1667.15	1619.72	1625];
    mapa_dist{4} = [1832.06	1769.1	1813.11	1821.56	1906.2	1756.65	1771.06	1742.96	1838.49	1853.67];
    mapa_dist{5} = [3329.46	3278.24	3293.75	3149.04	3163.78	3289.67	3241.7];
    mapa_dist{6} = [3963.84	3855.97	3874.78	3916.17	3762.22	3921.03	3980.46	3933.7	3905.36	3694.84];
    mapa_dist{7} = [5170.69	5417.15	5452.05	5051.2	5201.6	5181.1	5309.4	5510.1	5409.8	5513.2];
    mapa_dist{8} = [6050.68	5947.89	6160.72	6084.32	5762.06	5801.01	5659.67	5812.95	6222.15	6259.77];
    mapa_dist{9} = [6906.11	6332.67	6697.05	6649.94	7359.7	6951.02	7081.82	7609.52	6883.31	6711.24];
    mapa_dist{10} = [7532.82	8267.1	7978.65	8226.22	8010.48	8004.17	7873.71	8531.48 8179.85];
    
    mapa_distmin{1} = [232.15	200.85	218.57	200.85	200.85	200.85	230.36	229.95	210.75	200.85];
    mapa_distmin{2} = [637.09	743.31	677.6	674.8	694.09	694.09	676.6	686.73	718.61 677.1];
    mapa_distmin{3} = [1554.74	1595.8	1561.03	1636.05	1563.5	1608.13	1584.92	1658.87	1597.21	1568.77];
    mapa_distmin{4} = [1763.07	1769.1	1796.72	1816.8	1859.58	1712.69	1741.31	1742.96	1838.49	1836.18];
    mapa_distmin{5} = [3221.98	3226.03	3258.72	3112.89 3150.35	3278.55	3211.78];
    mapa_distmin{6} = [3922.84	3806.26	3840.21	3870.32	3738.67	3881.02	3980.46	3870.19	3835.57	3694.43];
    mapa_distmin{7} = [5169.26	5385.15	5309.55	5003.5	5201.6	5095.7	5291.8	5331.2	5252.9	5409.6];
    mapa_distmin{8} = [6026.75	5947.89	5936.02	6054.88	5762.06	5766.77	5633.94	5812.95	6128.67	6188.19];
    mapa_distmin{9} = [6864.71	6328.53	6668.77	6503.47	7316.81	6951.02	6982.87	7496.47	6868.67	6702.33];
    mapa_distmin{10} = [7439.17	8238.63	7924.51	8063.09	7909	7909	7873.71	8310.5 8117.76];
end
if(results==2)
    mapa_iter{1} = [305	350	160	213	275	180	475	293	312	214];
    mapa_iter{2} = [824	650	595	406	700	352	506	438	614 422];
    mapa_iter{3} = [525	466	940	522	491	503	617	1107	1045	1255];
    mapa_iter{4} = [1372	773	900	861	881	978	1036	968	854	968];
    mapa_iter{5} = [1813,724,722,866,795,609,742,1173,747,744];
    mapa_iter{6} = [1450	1264	1614	2031	1442	1765	1205	1577	1325	1248];
    mapa_iter{7} = [1628,1859,2536,1929,3467,4080,1362,3256,3007,3062];

    mapa_time{1} = [4	4.23	1.79	2.66	4.02	1.99	5.02	2.98	3.57	2.67];
    mapa_time{2} = [6.19	8.11	7.12	5.31	4.29	9.01	6.2	3.77	4.74	4.19];
    mapa_time{3} = [10.66	6.99	10.18	9.72	10.62	5.96	5.44	6.55	10.97	5.7];
    mapa_time{4} = [14.41	12.28	10.11	11.15	10.21	11.52	8.41	9.29	11.17	13.81];
    mapa_time{5} = [29.84	19.88	11.4	25.23	16.83	16.74	20.59	14.37	14.73	22.25];
    mapa_time{6} = [31.27	28.98	23.51	22.67	29.61	32.52	28.42	36.14	29.66	23.94];
    mapa_time{7} = [39.02	35.85	59.76	47.69	37.12	45.7	39.27	31.49	53.88	54.14];

    mapa_dist{1} = [189.6	178.06	203.66	180.13	180.13	178.06	216.85	199.77	178.06	178.06];
    mapa_dist{2} = [694.09	749.74	686.73	677.6	710.22	694.09	692.59	688.23	718.61 677.1];
    mapa_dist{3} = [1554.74	1606.64	1565.89	1636.05	1571.78	1608.13	1608.06	1667.15	1619.72	1625];
    mapa_dist{4} = [1832.06	1769.1	1813.11	1821.56	1906.2	1756.65	1771.06	1742.96	1838.49	1853.67];
    mapa_dist{5} = [2900 3000 2800];
    mapa_dist{6} = [3963.84	3855.97	3874.78	3916.17	3762.22	3921.03	3980.46	3933.7	3905.36	3694.84];
    mapa_dist{7} = [5170.69	5417.15	5452.05	5051.2	5201.6	5181.1	5309.4	5510.1	5409.8	5513.2];

    mapa_distmin{1} = [189.6	178.06	203.66	178.06	178.06	178.06	202.61	199.77	178.06	178.06];
    mapa_distmin{2} = [637.09	743.31	677.6	674.8	694.09	694.09	676.6	686.73	718.61 677.1];
    mapa_distmin{3} = [1554.74	1595.8	1561.03	1636.05	1563.5	1608.13	1584.92	1658.87	1597.21	1568.77];
    mapa_distmin{4} = [1763.07	1769.1	1796.72	1816.8	1859.58	1712.69	1741.31	1742.96	1838.49	1836.18];
    mapa_distmin{5} = [2800 2900 2700];
    mapa_distmin{6} = [3922.84	3806.26	3840.21	3870.32	3738.67	3881.02	3980.46	3870.19	3835.57	3694.43];
    mapa_distmin{7} = [5169.26	5385.15	5309.55	5003.5	5201.6	5095.7	5291.8	5331.2	5252.9	5409.6];
end
for i=1:n
   media_iter(i) = mean(mapa_iter{i});
   media_time(i) = mean(mapa_time{i});
   media_dist(i) = mean(mapa_dist{i});
   media_distmin(i) = mean(mapa_distmin{i});
   std_iter(i) = std(mapa_iter{i});
   std_time(i) = std(mapa_time{i});
   std_dist(i) = std(mapa_dist{i});
   std_distmin(i) = std(mapa_distmin{i});
end

figure(1)
errorbar(media_time,std_time,'x')
xlim([0 n+1])
grid on
xlabel('Valor do parâmetro n utilizado')
ylabel('Tempo de treinamento (s)')
title('Média e desvio padrão do tempo de treinamento em relação ao parâmetro n')

figure(2)
errorbar(media_iter,std_iter,'x')
xlim([0 n+1])
grid on
xlabel('Valor do parâmetro n utilizado')
ylabel('Número de iterações para convergência')
title('Média e desvio padrão do número de iterações do treinamento em relação ao parâmetro n')

figure(3)
errorbar(media_dist,std_dist,'x')
xlim([0 n+1])
grid on
xlabel('Valor do parâmetro n utilizado')
ylabel('Distância total percorrida (em unidades de grade)')
title('Média e desvio padrão da distância total percorrida em relação ao parâmetro n')

figure(4)
errorbar(media_distmin,std_distmin,'x')
xlim([0 n+1])
grid on
xlabel('Valor do parâmetro n utilizado')
ylabel('Distância total mínima encontrada (em unidades de grade)')
title('Média e desvio padrão da distância mínima encontrada em relação ao parâmetro n')

for i=1:n
    erro_percent(i) = (media_dist(i) - media_distmin(i))*100/media_distmin(i);
    erro_percent_max(i) = (media_dist(i) - min(mapa_distmin{i}))*100/min(mapa_distmin{i});
end
figure(5)
plot(1:n, erro_percent,'-x')
xlim([0 n+1])
grid on
xlabel('Valor do parâmetro n utilizado')
ylabel('Erro percentual')
title('Erro percentual médio entre a distância percorrida e a distância mínima encontrada em relação ao parâmetro n')

figure(6)
plot(1:n, 100*std_dist./media_dist,'-x')
xlim([0 n+1])
grid on
xlabel('Valor do parâmetro n utilizado')
ylabel('Erro percentual')
title('Desvio padrão percentual da distância total em relação ao parâmetro n')

figure(7)
plot(1:n, 100*std_distmin./media_distmin,'-x')
xlim([0 n+1])
grid on
xlabel('Valor do parâmetro n utilizado')
ylabel('Erro percentual')
title('Desvio padrão percentual da distância mínima encontrada em relação ao parâmetro n')

% figure(6)
% plot(1:n, erro_percent_max,'-x')
% xlim([0 7])
    
    
    
    
    
return


close all
warning off
clear wp

save_video = 0;
save_height = 0;
save_posxy = 0;
test_number = 5;
tol_wp = 5.5;

file = ['videos_testes/',num2str(test_number),'_data.txt'];
% file = ['C:\Users\walber jr\Documents\Tese\pyparrot-master\data_saved',num2str(test_number),'.txt'];
% file = ['C:\Users\walber jr\Documents\Tese\pyparrot-master\data_saved_final.txt'];
position = readtable(file);
f = figure();
axis([0 160 0 300])
% axis equal
axis square
grid on
hold on

%{
% fix 5_data
% for k = 1:9
%     if(str2num(position.Var2{k}(1))>1)
%         word1{k}(:) = position.Var2{k}(1:4);
%         word2{k}(:) = position.Var2{k}(5:7);
%     else
%         word1{k}(:) = position.Var2{k}(1:5);
%         word2{k}(:) = position.Var2{k}(6:8);
%     end
% end
% for k = 10:2293
%     if(str2num(position.Var2{k}(1))>1)
%         word1{k}(:) = position.Var2{k}(1:4);
%         word2{k}(:) = position.Var2{k}(5:8);
%     else
%         word1{k}(:) = position.Var2{k}(1:5);
%         word2{k}(:) = position.Var2{k}(6:9);
%     end
% end
% for k = 2294:2342
%     if(str2num(position.Var2{k}(1))>1)
%         word1{k}(:) = position.Var2{k}(1:4);
%         word2{k}(:) = position.Var2{k}(5:7);
%     else
%         word1{k}(:) = position.Var2{k}(1:5);
%         word2{k}(:) = position.Var2{k}(6:8);
%     end
% end
% fid = fopen('new_5_data.txt','wt');
% for i=1:length(word1)
%     fprintf(fid, [num2str(position.Var1(i)),',',word1{i},',',word2{i},'\n']);
% end
% fclose(fid);
%}
%remove zeros
a = position.Var1(position.Var1~=0.0);
b = position.Var2(position.Var2~=0.0);
c = position.Var3(position.Var3~=0.0);
clear position
position.Var1 = a;
position.Var2 = b;
position.Var3 = c;
%{
%xy for 3_data
% axis([0 350 100 400])
% hold on 
% grid on
% pos_init = [128.07, 253.85];
% wp(1,:) = [pos_init(1)-45, pos_init(2)+45];
% wp(2,:) = [pos_init(1), pos_init(2)+90];
% plot(pos_init(1),pos_init(2),'b*')
% plot(wp(1,1),wp(1,2),'k*')
% plot(wp(2,1),wp(2,2),'k*')
% a=[];
% b=[];
% j=1;
% for i=1:2:1962
%     a(j)=position.Var1(i);
%     b(j)=position.Var2(i);
%     j = j + 1;
%     plot(a,b,'b');
%     drawnow;
%     pause(0.03)
% end
% plot(a,b,'b');

%z
% axis([0 100])
% hold on 
% grid on
% a=[];
% b=[];
% j=1
% for i=2:2:1962
%     a(j)=position.Var1(i);
%     b(j)=j
%     j = j + 1;
%     plot(a,b,'b');
%     drawnow;
%     pause(0.05)
% end
% plot(a,b,'b');
%}

% video
% pos_init = [position.Var1(1), position.Var2(1)];
% pos_init = [120.86, 155.34-90];
% wp(1,:) = [pos_init(1)-45, pos_init(2)+45];
% wp(2,:) = [pos_init(1), pos_init(2)+90];
% plot(pos_init(1),pos_init(2),'b*')
% plot(wp(1,1),wp(1,2),'k*')
% plot(wp(2,1),wp(2,2),'k*')
% p(1) = plot(position.Var1(1:2),position.Var2(1:2),'b');
% for i = 1:length(position.Var1)-1 
%     p(i+1) = plot(position.Var1(1:i+1),position.Var2(1:i+1),'b');
%     delete(p(i))
%     drawnow
%     if(save_video)
%         F(i) = getframe(gcf);
%     end
%     pause(0.04)
% end

%plot all
plot(position.Var1,position.Var2,'b');
tol = 5.5;
axis equal
hold on
grid on
pos_init = [117.85, 81.55];
% pos_init = [128.07 , 163.85];
pos_init = [120.86, 155.34-90];
% wp(1,:) = [pos_init(1)-54, pos_init(2)];
% wp(2,:) = [pos_init(1)-54, pos_init(2)+54];
% wp(3,:) = [pos_init(1), pos_init(2)+54];
% wp(4,:) = [pos_init(1), pos_init(2)];

wp(1,:) = [pos_init(1)-45, pos_init(2)+45];
wp(2,:) = [pos_init(1), pos_init(2)+90];

h = viscircles(wp(1,:),tol, 'LineWidth', 1, 'LineStyle', '--');
h = viscircles(wp(2,:),tol, 'LineWidth', 1, 'LineStyle', '--');
% h = viscircles(wp(3,:),tol, 'LineWidth', 1, 'LineStyle', '--');
% h = viscircles(wp(4,:),tol, 'LineWidth', 1, 'LineStyle', '--');

axis([40 140 60 160])
axis([0 140 20 180])
plot(pos_init(1),pos_init(2),'g*')
plot(wp(1,1),wp(1,2),'k*')
plot(wp(2,1),wp(2,2),'k*')
% plot(wp(3,1),wp(3,2),'k*')
return
% height per time
h = figure()
h1 = plot(1:length(position.Var3),position.Var3,'b');
xlabel("tempo")
ylabel("altura (cm)")
title("Altura do drone durante o percurso")
grid on
if(save_height)
    saveas(h,[num2str(test_number),'_data_h.png']);
end

% x - y
h = figure()
h1 = plot(position.Var1,position.Var2,'b');
hold on
plot(pos_init(1),pos_init(2),'b*')
plot(wp(1,1),wp(1,2),'k*')
h1 = plot(position.Var1,position.Var2,'b');
xlabel("distância em x (cm)")
ylabel("distância em y (cm)")
title("Posições do drone durante percurso")
grid on
if(save_posxy)
    saveas(h,[num2str(test_number),'_data_xy.png']);
end

if(save_video)
    disp("saving...")
    video = VideoWriter([num2str(test_number),"_data"],"MPEG-4");
    video.Quality = 80;
    video.FrameRate = 20;
    open(video);
    writeVideo(video, F);
    close(video);
    disp("saved")
end