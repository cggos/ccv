%%����ά�ķ�ʽ��ʾ�Ҷ�ͼ��
I = imread('3.jpg');
p1 = rgb2gray(I);           % rgb תΪ�Ҷ�ͼ
[y,x]=size(p1);             % ȡ��ͼ���С
[X,Y]=meshgrid(1:x,1:y);    % ������������
pp=double(p1);              % uint8 ת��Ϊ double
mesh(X,Y,pp);               % ��ͼ
%colormap gray;              % ѡΪ�Ҷ�