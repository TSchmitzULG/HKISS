function kernels = computeKernel(noyauMesure,Amplitude,f1,R,fs)

N = length(noyauMesure);
B = (2*pi*f1*R/fs);
nbKernels = min(size(noyauMesure));

matrixCorrection = zeros(nbKernels,N);
for k = 1:nbKernels
    matrixCorrection(k,:) = (1/Amplitude^k) * exp(1i*(k-1)*B); 
end;

Atransform = volterraTransform(nbKernels);

%hammerstein kernels from measured kernels
kernels = ( Atransform * (matrixCorrection.*noyauMesure.')).';   

% Pour w < 0
 kernels(N/2+2:N,:)=conj(kernels(N/2:-1:2,:));
