clear;clc;close all;

% 1. diagnolize 
K = 5;      % dimension of problem

x_base = rand(1,K);     % generator vector
X = zeros(K,K);         % circulant matrix
for k=1:K
    X(k,:) = circshift(x_base, [0 k-1]);
end

x_hat = fft(x_base);    % DFT

F = transpose(dftmtx(K))/sqrt(K);       % the " ' " in matlab is transpose + conjugation

X2 = F*diag(x_hat)*F';

display(X);
display(real(X2));

% 2. fast compute correlation
C = X'*X;
C2 = (x_hat.*conj(x_hat))*conj(F)/sqrt(K);

display(C);
display(C2);