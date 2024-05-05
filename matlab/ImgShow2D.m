a=imread('2.png');
figure
imshow(a);

b=a(:,:,1);
figure
imshow(b);

c=b(50:end-20,40:end-50);
figure
imshow(c);