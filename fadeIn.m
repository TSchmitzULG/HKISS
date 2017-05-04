function [signal]=fadeIn(signal,R,loss)
fd1 = ceil(R*log(1+loss));

fade = (1-cos((0:fd1-1)/fd1*pi))/2;
index = 1:fd1;
signal(index) = signal(index).*fade;
