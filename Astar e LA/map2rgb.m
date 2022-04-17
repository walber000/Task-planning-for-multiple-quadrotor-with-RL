
function im = map2rgb(map)
%     map = imresize(map, [78, 78]);
    nx = size(map, 1);
    ny = size(map, 2);
    im = 255*ones(nx, ny, 3);
    im = cast(im, 'uint8');
%     vazio = 0;
    parede = 1;
    drone = 2;
    carga = 3;
    descarga = 4;
    % preto
    cor_parede = zeros(1, 1, 3);
    % azul
    cor_drone = zeros(1, 1, 3);
    cor_drone(1, 1, 3) = 255;
    % vermelho
    cor_carga = zeros(1, 1, 3);
    cor_carga(1, 1, 1) = 255;
    % verde
    cor_descarga = zeros(1, 1, 3);
    cor_descarga(1, 1, 2) = 255;
    px = 2;
    for i=1:nx
        for j=1:ny
            if(map(i, j) == parede)
                im(i,j, :) = cor_parede;
            end
            if(map(i, j) == drone)
                im((i-px):min(nx,(i+px)),(j-px):min(ny,(j+px)), :) = repmat(cor_drone, [length((i-px):min(nx,(i+px))),length((j-px):min(ny,(j+px)))]);
            end
            if(map(i, j) == carga)
                im((i-px):min(nx,(i+px)),(j-px):min(ny,(j+px)), :) = repmat(cor_carga, [length((i-px):min(nx,(i+px))),length((j-px):min(ny,(j+px)))]);
            end
            if(map(i, j) == descarga)
                im((i-px):min(nx,(i+px)),(j-px):min(ny,(j+px)), :) = repmat(cor_descarga, [length((i-px):min(nx,(i+px))),length((j-px):min(ny,(j+px)))]);
            end
        end
    end
    
%     im = imresize(im, [78, 78]);
%     Anew = zeros(48,48); % new matrix with size 48x48
%     s = size(A);
%     for jj = 1:s(1)
%         Anew(jj,:) = interp1(1:s(2),A(jj,:),linspace(1,s(2),size(A2,2)));
%     end
    
    imshow(im);
end