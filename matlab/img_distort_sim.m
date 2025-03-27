clear
clc

% 设定图像大小
w = 640;
h = 480;

% 设定焦距
fx = 500;
fy = 500;

% 设定主点
cx = (w-1)/2;
cy = (h-1)/2;

% 随机生成畸变
rng(100); %随机数种子
k1 = 0.1*randn(1);
k2 = 0.1*randn(1);
k3 = 0.1*randn(1);
k4 = 0.1*randn(1);
k5 = 0.1*randn(1);
k6 = 0.1*randn(1);
p1 = 0.001*randn(1);
p2 = 0.001*randn(1);

% 迭代次数
iter_num = 30;

% 采样步长，为了减少绘图点的密度
sampling_step = 8;

% 生成xd、yd
[ud, vd] = meshgrid(0:sampling_step:w-1,0:sampling_step:h-1);
xd = (ud-cx)/fx;
yd = (vd-cy)/fy;

% 去畸变迭代
x = xd;
y = yd;
for i = 1:iter_num
    r2 = x.^2+y.^2;
    x = (xd-(p1*(2*x.*y)+p2*(r2+2*x.^2)))./(1+k1*r2+k2*r2.^2+k3*r2.^3).*(1+k4*r2+k5*r2.^2+k6*r2.^3);
    y = (yd-(p1*(r2+2*y.^2)+p2*(2*x.*y)))./(1+k1*r2+k2*r2.^2+k3*r2.^3).*(1+k4*r2+k5*r2.^2+k6*r2.^3);
end

% 绘图
figure, hold on
scatter(xd(:),yd(:),'b.')
scatter(x(:),y(:),'r.')
legend('带畸变坐标(xd,yd)','去畸变坐标(x,y)')
axis ij
xlabel('x')
ylabel('y')

