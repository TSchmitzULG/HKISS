function freq = logSampleToFreq(sample,f1,f2,N)
% freq is the instantaneaous frequency of the nth sample of the logsweep
% for the parameters given
R = (N-1)/log(f2/f1);
freq = f1*exp((sample-1)/(R));