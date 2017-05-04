function [n] = logFreqToSample(freq,f1,f2,N)

R = (N-1)/log(f2/f1);
n = R*log(freq/f1)+1;