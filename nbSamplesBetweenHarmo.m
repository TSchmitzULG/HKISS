function [nbSamples,trueNbSamples] = nbSamplesBetweenHarmo(nbKernels,R)

OrderHarmo = 1:nbKernels;
trueNbSamples = R*log(OrderHarmo);
nbSamples = ceil(trueNbSamples);