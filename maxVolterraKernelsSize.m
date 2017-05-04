function volterraKernelsSize = maxVolterraKernelsSize(N,maxKernelsSize,nbKernels,deltaSamples,decayCut)

if nargin < 5
    decayCut = 0;
end;

volterraKernelsSize = zeros(1,nbKernels);

if maxKernelsSize-decayCut >= N
    volterraKernelsSize(1) = 2^(nextpow2(N)-1);
else
    volterraKernelsSize(1) = maxKernelsSize;
end;

for i = 2 : nbKernels
    if maxKernelsSize - decayCut < deltaSamples(i)-deltaSamples(i-1)
        volterraKernelsSize(i) = maxKernelsSize;
    else
        volterraKernelsSize(i) = 2^(nextpow2(deltaSamples(i)-deltaSamples(i-1))-1);
    end;
end;