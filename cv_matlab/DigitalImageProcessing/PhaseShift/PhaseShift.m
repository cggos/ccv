%%%%%%%%%%%%%%  四步相移法  %%%%%%%%%%%%%%
function [PhiReal,x] = PhaseShift()

% 竖直条纹图像
I_raw1 = imread('RasterImage/measImg5.bmp');
I_raw2 = imread('RasterImage/measImg6.bmp');
I_raw3 = imread('RasterImage/measImg7.bmp');
I_raw4 = imread('RasterImage/measImg8.bmp');

% I_raw1 = imread('20161128AM/new white board  340/measImg5.bmp');
% I_raw2 = imread('20161128AM/new white board  340/measImg6.bmp');
% I_raw3 = imread('20161128AM/new white board  340/measImg7.bmp');
% I_raw4 = imread('20161128AM/new white board  340/measImg8.bmp');

% I_raw1 = imread('RasterImage/4.bmp');
% I_raw2 = imread('RasterImage/5.bmp');
% I_raw3 = imread('RasterImage/6.bmp');
% I_raw4 = imread('RasterImage/7.bmp');

[W,H]=size(I_raw1);

% 400 * 300 区域
width = 600;
height = 400;
X_start = 100;
Y_start = 400;
X_range = [X_start:X_start+width];
Y_range = [Y_start:Y_start+height];
I_range1 = I_raw1(Y_range,X_range);
I_range2 = I_raw2(Y_range,X_range);
I_range3 = I_raw3(Y_range,X_range);
I_range4 = I_raw4(Y_range,X_range);

x = [1:width];

subplot(2,2,1);
plot(x,I_range1(20,x));
subplot(2,2,2);
plot(x,I_range2(20,x));
subplot(2,2,3);
plot(x,I_range3(20,x));
subplot(2,2,4);
plot(x,I_range4(20,x));
% 
figure
plot(x,I_range1(20,x),'r-',x,I_range2(20,x),'g-',x,I_range3(20,x),'b-',x,I_range4(20,x),'m-');
% figure
% plot(x,I_range2(20,x),'r-',x,I_range4(20,x),'g-');
% figure
% plot(x,I_range1(20,x),'b-',x,I_range3(20,x),'m-');

%%%%%%%%%%%%%% 根据 四步相移法 求图像相位主值
PhiReal = zeros(height,width);
%图像矩阵一般是uint8型的，范围是0~255的整数，把它转换为double型再减，否则得不到负数
I_numerator = double(I_range4)-double(I_range2);
I_denominator = double(I_range1)-double(I_range3);
% I_temp = (I_range4-I_range2)./(I_range1-I_range3);
PhiReal = pi+atan2(I_numerator,I_denominator);

% imwrite(PhiReal,'RasterImage/PhiReal.jpg');
valueLine = PhiReal(20,x);
figure
plot(x,valueLine);