function [hough_space,hough_circle,para] = hough_circle(BW,step_r,step_angle,r_min,r_max,p);
%[HOUGH_SPACE,HOUGH_CIRCLE,PARA] = HOUGH_CIRCLE(BW,STEP_R,STEP_ANGLE,R_MAX,P)
%------------------------------�㷨����-----------------------------
% ���㷨ͨ��a = x-r*cos(angle)��b = y-r*sin(angle)��Բͼ���еı�Ե��
% ӳ�䵽�����ռ�(a,b,r)�У�����������ͼ���Ҳ�ȡ�����꣬angle��r��ȡ
% һ���ķ�Χ�Ͳ���������ͨ������ѭ����angleѭ����rѭ�������ɽ�ԭͼ��
% �ռ�ĵ�ӳ�䵽�����ռ��У����ڲ����ռ䣨��һ�������С��������ɵ�
% ��������)��Ѱ��Բ�ģ�Ȼ������뾶���ꡣ
%-------------------------------------------------------------------

%------------------------------�������-----------------------------
% BW:��ֵͼ��
% step_r:����Բ�뾶����
% step_angle:�ǶȲ�������λΪ����
% r_min:��СԲ�뾶
% r_max:���Բ�뾶
% p:��p*hough_space�����ֵΪ��ֵ��pȡ0��1֮�����
%-------------------------------------------------------------------

%------------------------------�������-----------------------------
% hough_space:�����ռ䣬h(a,b,r)��ʾԲ����(a,b)�뾶Ϊr��Բ�ϵĵ���
% hough_circl:��ֵͼ�񣬼�⵽��Բ
% para:��⵽��Բ��Բ�ġ��뾶
%-------------------------------------------------------------------

% From Internet,Modified by mhjerry,2011-12-11

[m,n] = size(BW);
size_r = round((r_max-r_min)/step_r)+1;
size_angle = round(2*pi/step_angle);
 
hough_space = zeros(m,n,size_r);
 
[rows,cols] = find(BW);
ecount = size(rows);
 
% Hough�任
% ��ͼ��ռ�(x,y)��Ӧ�������ռ�(a,b,r)
% a = x-r*cos(angle)
% b = y-r*sin(angle)
for i=1:ecount
    for r=1:size_r
        for k=1:size_angle
            a = round(rows(i)-(r_min+(r-1)*step_r)*cos(k*step_angle));
            b = round(cols(i)-(r_min+(r-1)*step_r)*sin(k*step_angle));
            if(a>0&a<=m&b>0&b<=n)
                hough_space(a,b,r) = hough_space(a,b,r)+1;
            end
        end
    end
end
 
% ����������ֵ�ľۼ���
max_para = max(max(max(hough_space)));
index = find(hough_space>=max_para*p);
length = size(index);
hough_circle=zeros(m,n);
for i=1:ecount
    for k=1:length
        par3 = floor(index(k)/(m*n))+1;
        par2 = floor((index(k)-(par3-1)*(m*n))/m)+1;
        par1 = index(k)-(par3-1)*(m*n)-(par2-1)*m;
        if((rows(i)-par1)^2+(cols(i)-par2)^2<(r_min+(par3-1)*step_r)^2+5&...
                (rows(i)-par1)^2+(cols(i)-par2)^2>(r_min+(par3-1)*step_r)^2-5)
            hough_circle(rows(i),cols(i)) = 1;
        end
    end
end
 
% ��ӡ���
for k=1:length
    par3 = floor(index(k)/(m*n))+1;
    par2 = floor((index(k)-(par3-1)*(m*n))/m)+1;
    par1 = index(k)-(par3-1)*(m*n)-(par2-1)*m;
    par3 = r_min+(par3-1)*step_r;
    fprintf(1,'Center %d %d radius %d\n',par1,par2,par3);
    para(:,k) = [par1,par2,par3]';
end