function signalDelayed = delaySignal(x,delay)

X = fft(x);
N = length(x);
omega = linspace(0,2*pi,N+1);
omega = omega(1:end-1);
XDelayed = X.*exp(1j*omega*delay);
signalDelayed = ifft(XDelayed,'symmetric');