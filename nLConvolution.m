function [out] = nLConvolution(xM,tabImpulse)
%tabImpulse contains impulses responses in column
xSize = length(xM);
[hSize,nbnoyaux] = size(tabImpulse);
%Convolution length
sizeConv = 2^nextpow2(xSize+hSize-1);

OUT = zeros(sizeConv,1);

for k = 1:nbnoyaux
    X = fft(xM(:,k),sizeConv);
    H = fft(tabImpulse(:,k),sizeConv);
    OUT = OUT + X.*H ;
end;

out = (ifft(OUT,'symmetric'));