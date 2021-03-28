%%以三维的方式显示灰度图像
I = imread('3.jpg');
p1 = rgb2gray(I);           % rgb 转为灰度图
[y,x]=size(p1);             % 取出图像大小
[X,Y]=meshgrid(1:x,1:y);    % 生成网格坐标
pp=double(p1);              % uint8 转换为 double
mesh(X,Y,pp);               % 画图
%colormap gray;              % 选为灰度