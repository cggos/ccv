%%% ICP迭代最近点算法

function [T,revolveMatrix] = ICPiterator( sourcePoint , targetPoint )
%%% 获得匹配点集，重心
sourcePointCentre = getCentre(sourcePoint);
aimPointCentre = getCentre(targetPoint);

%%% 平移矩阵
T = getTranslation(aimPointCentre,sourcePointCentre);

%%% 中心化
midSourcePoint = centreTransform(sourcePoint, sourcePointCentre);
midAimPoint = centreTransform(targetPoint, aimPointCentre);

%%%旋转四元数
quaternion = getRevolveQuaternion(midSourcePoint,midAimPoint);

%%%旋转矩阵
revolveMatrix = getRevolveMatrix(quaternion);





%%%旋转四元数
function [quaternion] = getRevolveQuaternion( sourcePoint , targetPoint )
    %%% 协方差
    pp = sourcePoint * targetPoint';
    range = size(sourcePoint,1);
    pp = pp / range;
    
    %%% 反对称矩阵
    dissymmetryMatrix = pp - pp' ;
    
    %%% 列向量delta
    delta = [dissymmetryMatrix(2,3) ; dissymmetryMatrix(3,1) ; dissymmetryMatrix(1,2)];
    
    %%%对称矩阵Q
    Q = [ trace(pp) delta' ; delta   pp + pp' - trace(pp)*eye(3) ];
    
    %%%最大特征值，对应特征向量即为旋转四元数
    maxEigenvalues = max(eig(Q));
    quaternion = null(Q - maxEigenvalues*eye(length(Q)));

end

%%% 旋转矩阵
function [revolveMatrix] = getRevolveMatrix(quaternion)
    revolveMatrix = [ quaternion(1,1)^2 + quaternion(2,1)^2 - quaternion(3,1)^2 - quaternion(4,1)^2    2 * (quaternion(2,1)*quaternion(3,1) - quaternion(1,1)*quaternion(4,1))  2 * (quaternion(2,1)*quaternion(4,1) + quaternion(1,1)*quaternion(3,1));
                        2 * (quaternion(2,1)*quaternion(3,1) + quaternion(1,1)*quaternion(4,1))    quaternion(1,1)^2 - quaternion(2,1)^2 + quaternion(3,1)^2 - quaternion(4,1)^2     2 * (quaternion(3,1)*quaternion(4,1) - quaternion(1,1)*quaternion(2,1));
                        2 * (quaternion(2,1)*quaternion(4,1) - quaternion(1,1)*quaternion(3,1))  2 * (quaternion(3,1)*quaternion(4,1) + quaternion(1,1)*quaternion(2,1))   quaternion(1,1)^2 - quaternion(2,1)^2 - quaternion(3,1)^2 + quaternion(4,1)^2  ];
end

%%% 点集重心
function [centre] = getCentre( point )
    range = length(point);
    point1 = point';
    centre = sum(point1)/range;
    centre = centre';
end

%%% 获取平移矩阵
function [T] = getTranslation( aimPointCentre , sourcePointCentre )
    T = aimPointCentre - sourcePointCentre;
end

%%% 点集中心化
function [point] = centreTransform(point,centre)
range = size(point,1);
for i = 1:1:range
    point(:,i) = point(:,i) - centre;    
end
end

function [point] = counterCentreTransform(point,centre)
range = size(point,1);
for i = 1:1:range
    point(:,i) = point(:,i) + centre;    
end
end
