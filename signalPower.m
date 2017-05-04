function signalPowerFilterFs = signalPower(signal,upToPower,orderFilter)


signalPowerFilterFs(:,1) = signal;

for k=2:upToPower

    signalNewFsFiltered = resample(signal,k,1,orderFilter);
    
    signalPower = signalNewFsFiltered.^k;

    signalPowerFilterFs(:,k) = resample(signalPower,1,k,orderFilter);

end;

