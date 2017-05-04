function [sweep,R]=logSweep(N,f1,f2,loss,fs)
% N duration of the sweep in samples
% f1 initial frequency of the sweep in hz
% f2 final frequency of the sweep in hz


Amplitude = 1;
if nargin == 4
fs=44100;
end;
if nargin == 3
    loss = 0;
    fs=44100;
end;

n = 0:N-1;
R = (N-1)/(log(f2/f1));

phase = (f1*R/fs) * (exp(n/R)-1);
sweep = Amplitude*sin(2*pi*phase);

%fade in sweep
if(loss ~= 0)
sweep = fadeIn(sweep,R,loss(1));
sweep = fadeOut(sweep,R,f1,f2,N,loss(2));
end;

sweep=sweep';

