% Get occupancy grid from gazebo screenshot
function [BW] = gazimg2grid(filename)
    img = imread(filename);
    img = imresize(img, 0.2);
    hsvImage = rgb2hsv(img);
    hImage = hsvImage(:, :, 1);
    sImage = hsvImage(:, :, 2);
    vImage = hsvImage(:, :, 3);
    grayMask = sImage < 0.1; % or whatever....
    whitePixels = (vImage > 0.8) & grayMask; % or whatever.
    BW = bwmorph(whitePixels, 'close', 2);
    props = regionprops(BW, 'Area', 'BoundingBox');
    [~, idx] = max([props.Area]);
    bbox = round(props(idx).BoundingBox);
    BW = BW(bbox(2):bbox(2)+bbox(4)-1, ...
            bbox(1):bbox(1)+bbox(3)-1);
    BW = imrotate(BW, -90);
    BW = bwmorph(BW, 'dilate', 5);
    imshow(BW);
end
