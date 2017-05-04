function [InvSweep]=invSweepAn(len,R,f1,fs)
% N duration of invSweep
% R inverse frequency changing rate
% intial frequency of the sweep f1


if nargin == 3
fs=44100;
end;

L = R/fs;

%Analytical inverse sinesweep

f_ax =linspace(0,fs,len+1);
f_ax=f_ax(1:end-1);
InvSweep = 2*sqrt(f_ax/L).*exp(-1i*2*pi*(L*(f_ax.*(1-log(f_ax/f1))-f1)-1/8));
InvSweep = InvSweep.';
InvSweep(1)=0; 
%figure,viewBode(ifft(InvSweepA,'symmetric'))

