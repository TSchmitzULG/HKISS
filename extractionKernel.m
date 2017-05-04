function [tab] = extractionKernel(impulse,N,deltaSamples,trueDeltaSamples,volterraKernelsSize,nbKernels,offset,filter)

if nargin <7
    offset = 0;
    filter = 0;
end;
if nargin <8
    offset = 0;
end;
sizeIr = 2^nextpow2(volterraKernelsSize(1)); % IR + padding fft

tab = zeros(sizeIr,nbKernels);

debut = N+offset;
fin = debut+volterraKernelsSize(1)-1;

tab(1:(fin-debut)+1,1) = impulse(debut:fin);

for i=2:nbKernels
debut = N-deltaSamples(i)+offset;
fin = debut+volterraKernelsSize(i)-1;
impulseDelayed = delaySignal(impulse',deltaSamples(i)-trueDeltaSamples(i))';
if filter ~= 0
tab(1:(fin-debut)+1,i) = hann(fin-debut+1).*impulseDelayed(debut:fin);
else
 tab(1:(fin-debut)+1,i) = impulseDelayed(debut:fin);   
end;

end;