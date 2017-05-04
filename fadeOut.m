function [signal]=fadeOut(signal,R,f1,f2,N,loss)
%fade in de fadeTime second on the signal x
fd2 = N-ceil(R*log(f2*(1-loss)/f1)+1);

fade = (1-cos((0:fd2-1)/fd2*pi))/2;
index = (1:fd2)-1;
signal(end-index) = signal(end-index).*fade;

