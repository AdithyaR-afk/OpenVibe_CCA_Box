tic
%X = csvread('wdata.csv');
%Y = csvread('wref.csv');

%A = X;
%B = Y;
% A = csvread('samplecsva.csv');
% B = csvread('samplecsva.csv');
% A = A((1:20),(1:20));
% B = B((1:20),(1:20));

sti_f_ref = [15 , 10];
refSignals = ck_signal_windowed(sti_f_ref, 2, 250);
B = refSignals(:,:,1);

file='ssvep-switch-train-15Hz-Chethan-[2017.01.28-10.22.14].gdf';sti_f=15;
[s, h] = sload(file);
fs = h.SampleRate;
[s, h] = sload(file);
i = 1;
Ab(:,:) = s(1+((i-1)*25):500+(25*(i-1)),:);
A = permute(Ab,[2 1]);

%A = [4 5 6;4 5 6;5 6 7];
%B = [6 6 7;5 6 7;7 7 8];
Z = [A;B];
L = size(Z,1);
[M,N] = size(A);
Ua = mean(A.');
Ub = mean(B.');
U = [Ua,Ub];
Caa = zeros(2*M,2*M);
for k = 1:L
    for i = 1:2*M
        for j = 1:N
            Caa(k,i) = Caa(k,i) + (1/(N-1))*(Z(k,j)-U(k))*(Z(i,j)-U(i));
        end
    end
end
Cxx = zeros(M,M);
Cyy = zeros(M,M);
Cxy = zeros(M,M);

Cxx = Caa((1:M),(1:M)) ;
Cyy = Caa((M+1:2*M),(M+1:2*M));
Cxy = Caa((1:M),(M+1:2*M));
Cyx = Cxy';

Cxx = Cxx + 10^(-8)*eye(M);
Cyy = Cyy + 10^(-8)*eye(M);

iCxx= inv(Cxx);
iCyy = inv(Cyy);
Ax = iCxx*Cxy*iCyy*Cyx;

r = eig(iCxx*Cxy*iCyy*Cyx);
r = sqrt(r);
r = abs(r);
toc



