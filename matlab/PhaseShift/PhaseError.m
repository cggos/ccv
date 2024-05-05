[PhiReal,x] = PhaseShift();

% valueLine = PhiReal(200,x);
% figure
% plot(x,valueLine);
% 
% % % %%%%%%%%%%%%%% 求相位误差
% [N,n] = min(valueLine)
% for i = 2:width-1
%     if valueLine(i)-valueLine(i-1)<-6 && valueLine(i+1)-valueLine(i)<0.1
%         n=i;
%         N = valueLine(i);
%         break;
%     end
% end
% [M,m] = max(valueLine(n:n+300))
% m = m+n-1;
% 
% DeltaMN = m-n;
% valueT = valueLine(n:m);
% k = 2*pi/(DeltaMN);
% x = [0:m-n];
% Delta = valueT - k*x;
% figure
% hold on;
% plot(x,Delta)
% % % 最小二乘法拟合
% % p=polyfit(x,Delta,3);
% % plot(x,polyval(p,x),'r');

width = 600;
figure
hold on;
for row = 180:200
    valueLine = zeros(1,width);
    valueLine = PhiReal(row,x);
%     figure
%     plot(x,valueLine);

    % % %%%%%%%%%%%%%% 求相位误差
    [N,n] = min(valueLine);
    for i = 2:width-1
        if valueLine(i)-valueLine(i-1)<-6 && valueLine(i+1)-valueLine(i)<0.1
            n=i;
            N = valueLine(i);
            break;
        end
    end
    [M,m] = max(valueLine(n:n+300));
    m = m+n-1;

    DeltaMN = m-n;
    valueT = valueLine(n:m);
    k = 2*pi/(DeltaMN);
    y = [0:m-n];
    Delta = valueT - k*y;  
    
    for j = 1:m-n+1
        if abs(Delta(j)) > 2
            Delta(j) = Delta(j-1);
        end
    end
%     figure
    plot(y,Delta)
    % % 最小二乘法拟合
    % p=polyfit(x,Delta,3);
    % plot(x,polyval(p,x),'r');
end