 function [graph]=viewFft(signal,freq,range,fs,line,axesPosition)

 if nargin <5
     line = '-';
 end;

if(nargin<4)
fs=44100;
end;
x = signal(:);
L = 2^ceil(log2(length(x)));

XX = abs(fft(x,L));
XX = XX(1:L/2);
ww = [0:L/2-1]/L*fs;

if nargin==1

    graph = plot(ww,20*log10(XX));
    set(gca,'XScale','log');
    grid on,
    zoom on,
    xlabel('Frequency in Hz'),
    ylabel('Amplitude db');
   
   

elseif nargin==2
    graph = plot(ww,20*log10(XX));
     set(gca,'XScale','log');
    grid on;
    zoom on;
    xlabel('Frequency in Hz');
    ylabel('Amplitude db');
    axis([freq(1) freq(2) -60 20]);
  
elseif nargin==6
    graph = plot(axesPosition,ww,20*log10(XX),line);
    set(axesPosition,'XScale','log');
    grid(axesPosition,'on');
    zoom(axesPosition,'on');
    xlabel(axesPosition,'Frequency in Hz');
    ylabel(axesPosition,'Amplitude db');
    axis(axesPosition,[freq(1) freq(2) range(1) range(2)]);
else
    graph = plot(ww,20*log10(XX),line);
     set(gca,'XScale','log');
    grid on;
    zoom on;
    xlabel('Frequency in Hz');
    ylabel('Amplitude db');
    axis([freq(1) freq(2) range(1) range(2)]);
    
    
end;
