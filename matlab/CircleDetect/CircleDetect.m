function [accum, circen, cirrad] = CircleDetect(imggray)

img = imread(imggray);

% if isrgb(img)
%     imggray = rgb2gray(img);
% else if isgray(img)
%         imggray = img; 
%     else
%         imggray = img;
%     end
% end

imgsize = size(img);
if numel(imgsize)>2
    imggray = rgb2gray(img);
else
    imggray = img;
end

% imggray = im2bw(imggray, 0.1);

% %采用MATLAB中的函数filter2对受噪声干扰的图像进行均值滤波
% imggray = filter2(fspecial('average',3),imggray)/255; %模板尺寸为3

[accum, circen, cirrad] = CircularHough_Grd(imggray, [1 30],8, 3, 0.5);

% [circen, cirrad] = imfindcircles(imggray,[1 25], 'Sensitivity', .99);

if any(cirrad <= 0)
    inds = find(cirrad>0);
    cirrad = cirrad(inds);
    circen = circen(inds,:);
end

imshow(imggray);
hold on;
plot(circen(:,1), circen(:,2), 'r+');
for ii = 1 : size(circen, 1)
    rectangle('Position',[circen(ii,1) - cirrad(ii), circen(ii,2) - cirrad(ii), 2*cirrad(ii), 2*cirrad(ii)],...
        'Curvature', [1,1], 'edgecolor', 'b', 'linewidth', 1.5);
end
hold off;

