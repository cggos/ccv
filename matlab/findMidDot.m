%���ߣ��ߺ鳼  ѧ�ţ�120103050008  �༶���Զ���׿Խ�Ե��
%�������ҵ�ͼ(��ɫ������һ����ɫֱ��)��ֱ���е������
I_Orig=imread('3.jpg');
figure;
imshow(I_Orig);
I_R=I_Orig(:,:,1);
I_G=I_Orig(:,:,2);
I_B=I_Orig(:,:,3);
height=size(I_R,1);
width=size(I_R,2);
for row=1:height
    for col=1:width
        if I_R(row,col)<5
            I_R(row,col)=1;
        else
            I_R(row,col)=0;
        end
    end
end
for row=1:height
    for col=1:width
        if I_G(row,col)<5
            I_G(row,col)=1;
        else
            I_G(row,col)=0;
        end
    end
end
for row=1:height
    for col=1:width
        if I_B(row,col)<5
            I_B(row,col)=1;
        else
            I_B(row,col)=0;
        end
    end
end
I_Dest=I_R&I_G&I_B;
[rowVal,colVal]=find(I_Dest>0);
num=length(rowVal);
if mod(num,2)
    indexMid=(num+1)/2;
else
    indexMid=num/2;
end
disp('ֱ���е�ĺᡢ������ֱ�Ϊ��');
rowMid=rowVal(indexMid)
colMid=colVal(indexMid)